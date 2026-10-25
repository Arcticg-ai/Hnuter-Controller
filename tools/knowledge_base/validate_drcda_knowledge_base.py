#!/usr/bin/env python3
"""Audit and seal a generated knowledge bundle; never launch controller nodes."""
import argparse
import ast
import collections
import datetime as dt
import json
from pathlib import Path
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET

import package_drcda_knowledge_base as packaging


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--test-results-dir', type=Path, default=packaging.TASK_WORKSPACE)
    args = parser.parse_args()
    root = args.root.resolve()
    checks, failures = {}, []
    inventory = json.loads((root / '09_PROVENANCE/copy_inventory.json').read_text())
    missing = [v['path'] for v in inventory if not (root / v['path']).is_file()]
    checks['copy_inventory_missing_paths'] = missing
    failures.extend(missing)
    source_mismatch = []
    for item in inventory:
        source = Path(item['source'])
        if source.is_file() and packaging.sha(source) != item['source_sha256']:
            source_mismatch.append(str(source))
    checks['source_changed_after_copy'] = source_mismatch
    failures.extend(source_mismatch)
    syntax_count = 0
    for directory in ('03_CONTROLLER_REPO/controllers', '03_CONTROLLER_REPO/tools',
                      '03_CONTROLLER_REPO/tests', '08_TOOLS'):
        for path in (root / directory).rglob('*.py'):
            try:
                ast.parse(path.read_text(), filename=str(path))
                syntax_count += 1
            except SyntaxError as error:
                failures.append(str(error))
    checks['python_files_syntax_checked'] = syntax_count
    checks['symlinks'] = [str(p.relative_to(root)) for p in root.rglob('*') if p.is_symlink()]
    checks['private_dirs'] = [str(p.relative_to(root)) for p in root.rglob('*')
                             if p.is_dir() and p.name in {'.git', '.aws', '.codex', '.agents'}]
    failures.extend(checks['symlinks'] + checks['private_dirs'])
    key_patterns = dict(
        token_url=r'[?&]token=(?!\[REDACTED\])[A-Za-z0-9]{16,}',
        known_password=r'(?i)(?:密码|password)\s*(?:是|为|[:：=])?\s*[0-9]{3,}(?!\d)',
        private_key=r'-----BEGIN (?:OPENSSH |RSA |EC )?PRIVATE KEY-----')
    credential_hits, text_count = [], 0
    for path in root.rglob('*'):
        if not path.is_file() or path.suffix not in packaging.TEXT_EXT | {'.py', '.sh', '.in'} or path.stat().st_size > 16 * 1024 * 1024:
            continue
        content = path.read_text(errors='replace')
        text_count += 1
        for name, pattern in key_patterns.items():
            if re.search(pattern, content):
                credential_hits.append(dict(path=str(path.relative_to(root)), check=name))
    checks['credential_text_scan'] = dict(files=text_count, hits=credential_hits,
        boundary='Recognizable text credentials only; not a certification of binary ULog/parameter privacy.')
    failures.extend(str(v) for v in credential_hits)
    for name in ('tail_sitl_no_delay', 'hardware_main_no_delay'):
        sdf = root / '04_FIRMWARE' / name / 'Tools/simulation/gz/models/hnuter/model.sdf'
        forbidden = [token for token in ('servo_0_dynamic', 'transport_delay', 'FirstOrderActuator') if token in sdf.read_text()]
        checks[name + '_forbidden_tokens'] = forbidden
        failures.extend(forbidden)
    messages = [json.loads(line) for line in (root / '02_CONVERSATIONS/messages.jsonl').read_text().splitlines()]
    checks['conversation_roles'] = dict(collections.Counter(m['role'] for m in messages))
    if not set(checks['conversation_roles']) <= {'user', 'assistant'}:
        failures.append('Non-user-facing role in conversation export')
    checks['messages'] = len(messages)
    tests = {}
    for name in ('kb_regression_20261025.xml', 'kb_migrated_regression_20261025.xml'):
        path = args.test_results_dir / name
        shutil.copy2(path, root / '09_PROVENANCE' / name)
        suites = ET.parse(path).getroot().findall('testsuite')
        counts = {key: sum(int(s.get(key, '0')) for s in suites)
                  for key in ('tests', 'errors', 'failures', 'skipped')}
        tests[name] = counts
        if counts['errors'] or counts['failures']:
            failures.append('Regression failed: ' + name)
    checks['test_suites'] = tests
    proc = subprocess.run(['/usr/bin/python3', 'tools/build_hardware_drcda_v2.py', '--check'],
                          cwd=root / '03_CONTROLLER_REPO', capture_output=True, text=True)
    checks['standalone_generator'] = dict(returncode=proc.returncode, output=proc.stdout + proc.stderr)
    if proc.returncode:
        failures.append('Standalone generation mismatch')
    for name in ('tail_sitl_no_delay', 'hardware_main_no_delay'):
        checks[name + '_head'] = (root / '09_PROVENANCE' / name / 'HEAD.txt').read_text().strip()
    report = dict(validated_at=dt.datetime.now(dt.timezone.utc).isoformat(),
                  status='FAIL' if failures else 'PASS', checks=checks, failures=failures,
                  boundary='Offline source/package validation only. No remote host contact, Gazebo or hardware output.')
    packaging.write_json(root / '09_PROVENANCE/package_validation.json', report)
    shutil.copy2(__file__, root / '09_PROVENANCE/validate_drcda_knowledge_base.py')
    paths = sorted(p for p in root.rglob('*') if p.is_file() and p.name != 'SHA256SUMS')
    packaging.write(root / 'SHA256SUMS', '\n'.join(f'{packaging.sha(p)}  {p.relative_to(root)}' for p in paths))
    print(json.dumps(dict(status=report['status'], files=len(paths), messages=len(messages),
                          tests=tests, failures=failures), ensure_ascii=False))
    if failures:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
