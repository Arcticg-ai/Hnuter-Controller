#!/usr/bin/env python3
"""Curate a private, portable DRCDA knowledge base without touching checkouts."""
from __future__ import annotations

import argparse
import collections
import contextlib
import datetime as dt
import hashlib
import io
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tarfile
import zipfile

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1] if HERE.name == 'knowledge_base' else Path('/home/hnuter/px4_ws_ros2_consolidated')
TASK_WORKSPACE = Path('/home/hnuter/Documents/Codex/2026-10-11/drcda-gazebo')
ROS = Path('/home/hnuter/px4_ws_ros2')
SESSIONS = Path('/home/hnuter/.codex/sessions')
SITL_FIRMWARE = Path('/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl')
HARDWARE_FIRMWARE = Path('/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter')
SESSION_IDS = (
    '019fa36b-b8db-7d21-b9d9-25bc9e2dbcb1',
    '019fcd96-a556-7032-9960-6f4d0f8786ba',
    '019fcdd2-a108-7602-8a53-234064c41daa',
    '019ffbe8-3318-7e02-b12c-15726b0e06cd',
    '01a0e7a0-3994-70f2-b432-a322c51cf6ff',
    '01a0e7a1-c2d7-76f2-b37f-d2f3da64313a',
)
SKIP_PARTS = {'.git', '.pytest_cache', '__pycache__', '.vscode', '.aws', '.codex', '.agents'}
TEXT_EXT = {'.md', '.txt', '.json', '.jsonl', '.log', '.patch', '.diff', '.xml', '.yaml', '.yml', '.csv'}
ARCHIVES = ('.tar.gz', '.tar', '.tgz', '.zip', '.zst')
KNOWN_SECRET_VALUES = set()


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def redact(text):
    KNOWN_SECRET_VALUES.update(re.findall(r'(?i)[?&]token=([A-Za-z0-9_.~+/=-]{16,})', text))
    for secret in KNOWN_SECRET_VALUES:
        text = text.replace(secret, '[REDACTED]')
    text = re.sub(r'([?&]token=)[^\s&"<>`]+', r'\1[REDACTED]', text, flags=re.I)
    text = re.sub(r'(?i)(sshpass\s+-p\s+)(?:\x27[^\x27]*\x27|"[^"]*"|\S+)',
                  r'\1[REDACTED]', text)
    text = re.sub(r'(?im)((?:密码|password)\s*(?:是|为|[:：=])?\s*)[\x27"]?[0-9]{3,}[\x27"]?(?!\d)',
                  r'\1[REDACTED]', text)
    text = re.sub(r'(?i)(Bearer\s+)[A-Za-z0-9_.~+/=-]{12,}', r'\1[REDACTED]', text)
    text = re.sub(r'(?i)((?:access_token|api_key|auth_token|web_token)\s*["\x27]?\s*[:=]\s*["\x27]?)[A-Za-z0-9_.~+/=-]{16,}',
                  r'\1[REDACTED]', text)
    text = re.sub(r'(?i)((?:网页访问令牌|访问令牌|网页令牌|token)\s*(?:为|是|[:：=])?\s*[`"\x27]?)[A-Fa-f0-9]{24,64}\b',
                  r'\1[REDACTED]', text)
    text = re.sub(r'-----BEGIN (?:OPENSSH |RSA |EC )?PRIVATE KEY-----.*?-----END (?:OPENSSH |RSA |EC )?PRIVATE KEY-----',
                  '[REDACTED PRIVATE KEY]', text, flags=re.S)
    return text


def write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.rstrip() + '\n', encoding='utf-8')


def write_json(path, value):
    write(path, json.dumps(value, ensure_ascii=False, indent=2))


def git(path, *args):
    p = subprocess.run(['git', '-C', str(path), *args], capture_output=True, text=True)
    return p.stdout if p.returncode == 0 else 'UNAVAILABLE: ' + p.stderr


class Builder:
    def __init__(self, root):
        self.root = root
        self.copied = []
        self.excluded = []
        self.archives = []

    def copy_file(self, src, dest, *, source=None):
        dest.parent.mkdir(parents=True, exist_ok=True)
        original = sha(src)
        if src.suffix.lower() in TEXT_EXT and src.stat().st_size < 16 * 1024 * 1024:
            try:
                content = src.read_text(encoding='utf-8')
                clean = redact(content)
            except UnicodeError:
                clean = content = None
            if clean is not None and clean != content:
                write(dest, clean)
            else:
                shutil.copy2(src, dest)
        else:
            shutil.copy2(src, dest)
        self.copied.append(dict(path=str(dest.relative_to(self.root)),
                                source=str(source or src), source_sha256=original,
                                packaged_sha256=sha(dest), redacted=original != sha(dest)))

    def copy_tree(self, src, dest, *, exclude_archives=False):
        for parent, dirs, files in os.walk(src, followlinks=False):
            dirs[:] = sorted(d for d in dirs if d not in SKIP_PARTS)
            for name in sorted(files):
                path = Path(parent) / name
                relative = path.relative_to(src)
                if path.is_symlink():
                    self.excluded.append(dict(source=str(path), reason='symlink; not portable'))
                    continue
                if name.endswith(('.pyc', '.pyo')) or name in {'.DS_Store', 'web_drcda_parameters_20261024.png'}:
                    self.excluded.append(dict(source=str(path), reason='cache or token-bearing UI image'))
                    continue
                if name.endswith(ARCHIVES):
                    if exclude_archives:
                        self.excluded.append(dict(source=str(path), reason='duplicate nested evidence archive; expanded evidence preferred'))
                    else:
                        self.archives.append((path, dest / relative.parent / (name + '.expanded')))
                    continue
                self.copy_file(path, dest / relative)

    def expand_archives(self):
        for src, dest in self.archives:
            members = []
            if zipfile.is_zipfile(src):
                with zipfile.ZipFile(src) as archive:
                    for m in archive.infolist():
                        if not m.is_dir():
                            members.append((m.filename, archive.read(m)))
            else:
                with tarfile.open(src, 'r:*') as archive:
                    for m in archive:
                        if m.isfile():
                            f = archive.extractfile(m)
                            if f:
                                members.append((m.name, f.read()))
            inventory = []
            for name, data in members:
                p = Path(name)
                if p.is_absolute() or '..' in p.parts or any(part in SKIP_PARTS for part in p.parts):
                    self.excluded.append(dict(source=f'{src}!{name}', reason='unsafe/private/cache archive member'))
                    continue
                if name.endswith(ARCHIVES) or name.endswith(('.pyc', '.pyo')):
                    self.excluded.append(dict(source=f'{src}!{name}', reason='nested archive or cache member'))
                    continue
                target = dest / p
                target.parent.mkdir(parents=True, exist_ok=True)
                if p.suffix.lower() in TEXT_EXT | {'.py', '.sh', '.in'}:
                    try:
                        data = redact(data.decode('utf-8')).encode('utf-8')
                    except UnicodeError:
                        pass
                target.write_bytes(data)
                inventory.append(dict(member=name, sha256=sha(target)))
            write_json(dest / '_ARCHIVE_SOURCE.json', dict(source=str(src), sha256=sha(src), members=inventory))

    def provenance(self, source, name):
        dest = self.root / '09_PROVENANCE' / name
        dest.mkdir(parents=True)
        for filename, command in (
            ('HEAD.txt', ('rev-parse', 'HEAD')),
            ('branch.txt', ('branch', '--show-current')),
            ('status.txt', ('status', '--short', '--branch')),
            ('remotes.txt', ('remote', '-v')),
            ('recent_commits.txt', ('log', '-15', '--format=fuller')),
            ('working_tree.patch', ('diff', 'HEAD', '--binary')),
            ('untracked_paths.txt', ('ls-files', '--others', '--exclude-standard')),
        ):
            write(dest / filename, redact(git(source, *command)))

    def controller_repo(self):
        dest = self.root / '03_CONTROLLER_REPO'
        for name in ('controllers', 'tools', 'tests', 'config', 'docs', 'reports'):
            self.copy_tree(REPO / name, dest / name)
        for name in ('README.md', 'requirements-px4-venv.txt', '.gitignore', '.gitmodules'):
            self.copy_file(REPO / name, dest / name)
        # Preserve unsupported old settings as history, not active launch choices.
        for path in (dest / 'config/simulation').glob('identified_delay*.json'):
            historical = self.root / '06_REFERENCES/historical_delay_configs_NOT_FOR_VALIDATION' / path.name
            historical.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(path, historical)
            for item in self.copied:
                if item['path'] == str(path.relative_to(self.root)):
                    item['path'] = str(historical.relative_to(self.root))
        self.provenance(REPO, 'controller_repo')

    def firmware(self):
        for name, path in (
            ('tail_sitl_no_delay', SITL_FIRMWARE),
            ('hardware_main_no_delay', HARDWARE_FIRMWARE),
        ):
            dest = self.root / '04_FIRMWARE' / name
            files = [
                'Tools/simulation/gz/models/hnuter',
                'Tools/simulation/gz/worlds/default.sdf',
                'Tools/simulation/gz/server.config',
                'ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter',
                'src/modules/mc_rate_control/HnuterControl.cpp',
                'src/modules/mc_rate_control/HnuterControl.hpp',
                'src/modules/mc_rate_control/hnuter_control_params.c',
                'src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHnuter.cpp',
                'src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHnuter.hpp',
                'src/modules/control_allocator/VehicleActuatorEffectiveness/HnuterPositionControl.cpp',
                'src/modules/control_allocator/VehicleActuatorEffectiveness/HnuterPositionControl.hpp',
                'src/modules/land_detector/MulticopterLandDetector.cpp',
                'src/modules/land_detector/MulticopterLandDetector.h',
                'src/modules/land_detector/HnuterDirectLandingGuard.hpp',
                'src/modules/simulation/gz_bridge/GZBridge.cpp',
                'src/modules/simulation/gz_bridge/server.config',
                'src/modules/uxrce_dds_client/dds_topics.yaml',
                'msg/VehicleStatus.msg', 'msg/OffboardControlMode.msg',
            ]
            for name_in in files:
                source = path / name_in
                if source.is_dir():
                    self.copy_tree(source, dest / name_in)
                elif source.is_file():
                    self.copy_file(source, dest / name_in)
            binary = path / 'build/px4_sitl_default/bin/px4'
            if binary.exists():
                write_json(dest / 'binary_identity_ONLY.json', dict(path=str(binary), sha256=sha(binary),
                           note='Identity only. Binary/build tree not distributed; this is not a full firmware checkout.'))
            self.provenance(path, name)

    def conversations(self):
        dest = self.root / '02_CONVERSATIONS'
        indices, merged = [], {}
        all_paths = list(SESSIONS.rglob('*.jsonl*'))
        for sid in SESSION_IDS:
            paths = [p for p in all_paths if sid in p.name]
            if not paths:
                indices.append(dict(session_id=sid, error='source not found'))
                continue
            path = max(paths, key=lambda p: p.stat().st_size)
            messages, skipped, bad_lines = [], collections.Counter(), 0
            with contextlib.ExitStack() as stack:
                if path.suffix == '.zst':
                    proc = subprocess.Popen(['zstd', '-qdc', str(path)], stdout=subprocess.PIPE)
                    stack.callback(proc.wait)
                    stream = stack.enter_context(io.TextIOWrapper(proc.stdout, encoding='utf-8'))
                else:
                    stream = stack.enter_context(path.open(encoding='utf-8'))
                for line in stream:
                    try:
                        item = json.loads(line)
                    except json.JSONDecodeError:
                        bad_lines += 1
                        continue
                    p = item.get('payload', {})
                    if item.get('type') != 'response_item' or p.get('type') != 'message':
                        continue
                    role = p.get('role')
                    if role not in {'user', 'assistant'} or p.get('channel') in {'analysis', 'summary'}:
                        skipped['not_user_facing'] += 1
                        continue
                    parts = [v.get('text', '') for v in p.get('content', []) if v.get('type') in {'input_text', 'output_text'}]
                    content = '\n'.join(parts).strip()
                    # Learn URL credentials before stripping ambient browser blocks.
                    redact(content)
                    for tag in ('environment_context', 'in-app-browser-context', 'recommended_plugins', 'external_codex_apps_open_page'):
                        content = re.sub(fr'<{tag}\b[^>]*>.*?</{tag}>', '', content, flags=re.S).strip()
                    if not content:
                        skipped['empty_or_environment_only'] += 1
                        continue
                    # Some session records have a visible text plus an image/audio block.
                    media_count = sum(v.get('type') not in {'input_text', 'output_text'} for v in p.get('content', []))
                    message = dict(timestamp=item.get('timestamp', ''), role=role,
                                   text=redact(content), session_id=sid, nontext_blocks=media_count)
                    messages.append(message)
                    key = (message['timestamp'], role, message['text'])
                    if key not in merged:
                        merged[key] = dict(message, source_sessions=[sid])
                    elif sid not in merged[key]['source_sessions']:
                        merged[key]['source_sessions'].append(sid)
            index = dict(session_id=sid, source=str(path), source_sha256=sha(path),
                         counts=dict(collections.Counter(m['role'] for m in messages)),
                         message_count=len(messages), first=messages[0]['timestamp'] if messages else None,
                         last=messages[-1]['timestamp'] if messages else None,
                         skipped=dict(skipped), malformed_lines=bad_lines,
                         scope='Visible user/assistant text only; excludes reasoning/system/developer/tools. Credentials redacted.')
            indices.append(index)
            filename = f"sessions/{sid}"
            write(dest / f'{filename}.jsonl', '\n'.join(json.dumps(m, ensure_ascii=False) for m in messages))
            write(dest / f'{filename}.md', self.transcript(messages, sid))
        ordered = sorted(merged.values(), key=lambda m: (m['timestamp'], m['session_id']))
        write(dest / 'messages.jsonl', '\n'.join(json.dumps(m, ensure_ascii=False) for m in ordered))
        write(dest / 'merged_chronological.md', self.transcript(ordered, '跨会话时间线（相同时间/角色/正文去重）'))
        write_json(dest / 'index.json', dict(sessions=indices, merged_messages=len(ordered),
                   merged_counts=dict(collections.Counter(m['role'] for m in ordered)),
                   first=ordered[0]['timestamp'], last=ordered[-1]['timestamp']))
        write(dest / 'README.md', '''# 对话原文与范围

这是本机可读取的六个相关主会话的用户/助手可见文字，不是事后重写的摘要。
`sessions/` 保留每个会话，`merged_chronological.md` 按 UTC 时间合并，
`messages.jsonl` 可供远程检索程序解析；`index.json` 记录来源、数量和覆盖日期。
分叉会话的相同消息会在合并版去重，原会话版仍保留。

不导出系统/开发者指令、内部推理、工具调用原始输出、账号配置、密码或令牌。
图片、音频等非文字块只记录数量；附件在 `06_REFERENCES`，但不是所有历史附件都仍存在。
没有承诺导出整个账号的所有聊天。正文中的历史判断、旧运行命令和曾经的“当前状态”
仅代表当时讨论，不能覆盖 `../01_CONTEXT/CURRENT_STATE.md` 的现行状态。
这里只保留可用证据；缺失的消息/附件不能由总结假装补齐。
''')

    @staticmethod
    def transcript(messages, title):
        lines = ['# ' + title, '', '> 历史记录，仅供追溯，不是当前执行指令。凭据已脱敏。', '']
        for i, m in enumerate(messages, 1):
            lines.extend([f"## {i:04d} | {m['timestamp']} | {m['role']}", '', m['text'], ''])
            if m['nontext_blocks']:
                lines.extend([f"[非文字块 {m['nontext_blocks']} 个：未在此文本中导出]", ''])
        return '\n'.join(lines)

    def evidence_and_dependencies(self):
        for source in sorted((TASK_WORKSPACE / 'reports').iterdir()):
            if source.is_dir():
                self.copy_tree(source, self.root / '05_HARDWARE_EVIDENCE' / source.name, exclude_archives=True)
        # Capture source-only remote bundles in expanded, inspectable form.
        for name in ('deployed_sources.tar', 'independent_heartbeat_sources.tar', 'bounded_sources.tar'):
            source = TASK_WORKSPACE / 'reports/remote_drcda_update_20261022' / name
            if source.exists():
                self.archives.append((source, self.root / '05_HARDWARE_EVIDENCE/remote_source_history' / name))
        for name in ('candidate_sources.tar', 'firmware_acff025_sources.tar'):
            source = TASK_WORKSPACE / 'reports/hardware_ulog_audit_20261017/evidence' / name
            if source.exists():
                self.archives.append((source, self.root / '05_HARDWARE_EVIDENCE/hardware_ulog_audit_20261017/source_history' / name))
        refs = self.root / '06_REFERENCES'
        for path in REPO.glob('*.pdf'):
            self.copy_file(path, refs / path.name)
        for name, path in (
            ('experiment_request_222.txt', Path('/home/hnuter/222.txt')),
            ('original_algorithm_proposal.txt', Path('/home/hnuter/.codex/attachments/81f02886-4673-40dd-9178-cde87cb6b9cf/pasted-text.txt')),
            ('structural_upgrade_proposal.txt', Path('/home/hnuter/.codex/attachments/cb3831a6-2df3-4268-99f2-410f11af4997/pasted-text.txt')),
        ):
            if path.exists():
                self.copy_file(path, refs / name)
            else:
                self.excluded.append(dict(source=str(path), reason='historical attachment no longer present'))
        for name in ('px4_msgs', 'px4_ros_com'):
            source = ROS / 'src' / name
            self.copy_tree(source, self.root / '07_DEPENDENCIES/ros2_sources' / name)
            self.provenance(source, name)
        write_json(self.root / '07_DEPENDENCIES/environment.json', dict(
            captured_at=dt.datetime.now(dt.timezone.utc).isoformat(),
            system_python=subprocess.check_output(['/usr/bin/python3', '--version'], text=True).strip(),
            ros_setup_present=Path('/opt/ros/jazzy/setup.bash').exists(),
            system_packages=subprocess.run(['/usr/bin/python3', '-m', 'pip', 'list', '--format=json'],
                                           capture_output=True, text=True).stdout,
            note='Observed local environment, not a portable venv or exact remote lockfile.'))

    def legacy_local(self):
        dest = self.root / '10_LEGACY_LOCAL'
        for name in ('hnuter_external_direct_drcda_hardware.py',
                     'hnuter_external_direct_controller_hardware.py',
                     'hnuter_external_direct_controller_debug.py',
                     'hnuter_log_paths.py', 'README.md',
                     'tests/test_hnuter_external_direct_drcda_hardware.py'):
            path = ROS / name
            if path.is_file():
                self.copy_file(path, dest / 'old_workspace_source' / name)
        self.copy_tree(ROS / 'config', dest / 'old_workspace_source/config')
        inventory = []
        base = ROS / 'hnuter_logs'
        for parent, dirs, files in os.walk(base):
            dirs[:] = sorted(d for d in dirs if d not in SKIP_PARTS)
            for name in sorted(files):
                path = Path(parent) / name
                if path.is_symlink():
                    continue
                rel = path.relative_to(base)
                include = (path.suffix in {'.md', '.pdf'} or
                           (path.suffix == '.json' and any(word in name for word in ('manifest', 'metrics', 'summary'))))
                inventory.append(dict(path=str(rel), bytes=path.stat().st_size,
                                      included=include, reason='historical report/index' if include else 'large legacy raw/log/media omitted'))
                if include:
                    self.copy_file(path, dest / 'historical_reports_NOT_CURRENT_EVIDENCE' / rel)
        write_json(dest / 'old_log_inventory.json', dict(source=str(base), files=inventory,
                   note='Inventory only for omitted raw data. Not a full backup of the old 5.4 GiB log directory.'))
        self.provenance(ROS, 'legacy_workspace')

    def finish_manifest(self):
        for path in self.root.rglob('*'):
            if path.is_file() and path.suffix in TEXT_EXT and path.stat().st_size < 16 * 1024 * 1024:
                try:
                    before = path.read_text()
                except UnicodeError:
                    continue
                after = redact(before)
                if before != after:
                    write(path, after)
        for item in self.copied:
            target = self.root / item['path']
            item['packaged_sha256'] = sha(target)
            item['redacted'] = item['source_sha256'] != item['packaged_sha256']
        write_json(self.root / '09_PROVENANCE/copy_inventory.json', self.copied)
        write_json(self.root / '09_PROVENANCE/exclusions.json', self.excluded)
        paths = sorted(p for p in self.root.rglob('*') if p.is_file() and p.name != 'SHA256SUMS')
        write(self.root / 'SHA256SUMS', '\n'.join(f'{sha(p)}  {p.relative_to(self.root)}' for p in paths))


def main():
    global REPO, ROS, SESSIONS, TASK_WORKSPACE, SITL_FIRMWARE, HARDWARE_FIRMWARE, SESSION_IDS
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repo', type=Path, default=REPO)
    parser.add_argument('--legacy-workspace', type=Path, default=ROS)
    parser.add_argument('--task-workspace', type=Path, default=TASK_WORKSPACE)
    parser.add_argument('--sessions-root', type=Path, default=SESSIONS)
    parser.add_argument('--sitl-firmware', type=Path, default=SITL_FIRMWARE)
    parser.add_argument('--hardware-firmware', type=Path, default=HARDWARE_FIRMWARE)
    parser.add_argument('--session-id', action='append', help='Explicit visible conversation IDs; defaults to the six recorded project sessions')
    args = parser.parse_args()
    REPO, ROS, SESSIONS = (p.expanduser().resolve() for p in (args.repo, args.legacy_workspace, args.sessions_root))
    TASK_WORKSPACE = args.task_workspace.expanduser().resolve()
    SITL_FIRMWARE = args.sitl_firmware.expanduser().resolve()
    HARDWARE_FIRMWARE = args.hardware_firmware.expanduser().resolve()
    if args.session_id:
        SESSION_IDS = tuple(args.session_id)
    for source in (REPO, ROS, SESSIONS, TASK_WORKSPACE / 'reports', SITL_FIRMWARE, HARDWARE_FIRMWARE):
        if not source.is_dir():
            parser.error(f'Missing source directory: {source}')
    output = args.output.expanduser().resolve()
    if any(output == source or source.is_relative_to(output) for source in (REPO, ROS, SESSIONS, TASK_WORKSPACE)):
        parser.error('Output must not replace or contain a source directory')
    if output.is_relative_to(REPO / 'tools') or output.is_relative_to(REPO / 'reports'):
        parser.error('Output must be outside copied source directories')
    args.output = output
    args.output.mkdir(parents=True, exist_ok=False)
    builder = Builder(args.output.resolve())
    print('Copying current source, tests, configs and complete expanded report data...', flush=True)
    builder.controller_repo()
    print('Recording firmware identity and selected no-delay firmware sources...', flush=True)
    builder.firmware()
    print('Exporting six related visible conversation histories...', flush=True)
    builder.conversations()
    print('Copying hardware evidence, attachments and compatible ROS message sources...', flush=True)
    builder.evidence_and_dependencies()
    print('Adding legacy hardware source and historical report-only indices...', flush=True)
    builder.legacy_local()
    print('Expanding historical source archives with path checks and redaction...', flush=True)
    builder.expand_archives()
    shutil.copytree(HERE / 'knowledge_base_template', builder.root, dirs_exist_ok=True)
    shutil.copy2(__file__, builder.root / '09_PROVENANCE/package_drcda_knowledge_base.py')
    for path in (Path('/tmp/hnuter_pose_handover_tests.xml'),):
        if path.exists():
            builder.copy_file(path, builder.root / '09_PROVENANCE/historical_validation' / path.name)
    write_json(builder.root / '09_PROVENANCE/package_metadata.json', dict(
        packaged_at=dt.datetime.now(dt.timezone.utc).isoformat(),
        controller_head=git(REPO, 'rev-parse', 'HEAD').strip(),
        hardware_entry_sha256=sha(REPO / 'controllers/hardware/hnuter_external_direct_drcda_v2_hardware.py'),
        policy='no-delay firmware only; private local migration; no remote connection, upload, push or actuator launch',
        source_snapshot='current working tree, including relevant tracked edits and untracked files',
        historical_report_boundary='Original reports are retained; generated top-level guides identify superseded conclusions.'))
    builder.finish_manifest()
    print(json.dumps(dict(root=str(builder.root), copied=len(builder.copied),
                          files=sum(p.is_file() for p in builder.root.rglob('*'))), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
