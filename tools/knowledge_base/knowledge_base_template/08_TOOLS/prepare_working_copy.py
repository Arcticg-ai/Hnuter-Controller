#!/usr/bin/env python3
"""Make a new working copy; never overwrite source or launch any vehicle code."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--firmware', type=Path)
    parser.add_argument('--include-reports', action='store_true', help='Also duplicate the large evidence tree')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    source = root / '03_CONTROLLER_REPO'
    output = args.output.expanduser().resolve()
    if output.exists() or output.is_relative_to(root):
        parser.error('Use a new directory outside the immutable knowledge bundle')
    firmware = args.firmware.expanduser().resolve() if args.firmware else output / '_NO_DELAY_FIRMWARE_NOT_CONFIGURED'
    if args.firmware:
        sdf = firmware / 'Tools/simulation/gz/models/hnuter/model.sdf'
        if not sdf.is_file() or not (firmware / 'Makefile').is_file() or not (firmware / 'CMakeLists.txt').is_file():
            parser.error('A complete no-delay firmware checkout with Hnuter SDF is required')
        if any(token in sdf.read_text() for token in ('servo_0_dynamic', 'transport_delay', 'FirstOrderActuator')):
            parser.error('Refusing delayed firmware')
    ignore = shutil.ignore_patterns('__pycache__', '.pytest_cache', '.git')
    output.mkdir(parents=True)
    for path in source.iterdir():
        if path.name == 'reports' and not args.include_reports:
            continue
        target = output / path.name
        if path.is_dir():
            shutil.copytree(path, target, ignore=ignore)
        else:
            shutil.copy2(path, target)
    replacements = [
        ('/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl', str(firmware)),
        ('/home/hnuter/px4_ws_ros2_consolidated', str(output)),
        ('/home/hnuter/px4_ws_ros2/px4-venv/bin/python', '/usr/bin/python3'),
        ('/home/hnuter/px4_ws_ros2', str(output)),
    ]
    changed = []
    for directory in ('controllers', 'tools', 'tests'):
        for path in (output / directory).rglob('*'):
            if path.suffix not in {'.py', '.sh'} or not path.is_file():
                continue
            old = path.read_text()
            new = old
            for before, after in replacements:
                new = new.replace(before, after)
            if new != old:
                path.write_text(new)
                changed.append(dict(path=str(path.relative_to(output)),
                                    before_sha256=hashlib.sha256(old.encode()).hexdigest(),
                                    after_sha256=hashlib.sha256(new.encode()).hexdigest()))
    for name in ('px4_msgs', 'px4_ros_com'):
        shutil.copytree(root / '07_DEPENDENCIES/ros2_sources' / name,
                        output / 'src' / name, ignore=ignore)
    (output / 'MIGRATION_PATH_CHANGES.json').write_text(json.dumps(dict(
        knowledge_bundle=str(root), firmware=str(firmware), firmware_configured=bool(args.firmware),
        changes=changed, note='Mechanical path substitution only; no controller launched.'), indent=2) + '\n')
    print(json.dumps(dict(output=str(output), changed_files=len(changed),
                          firmware_configured=bool(args.firmware), control_started=False), ensure_ascii=False))


if __name__ == '__main__':
    main()
