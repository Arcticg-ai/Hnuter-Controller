#!/usr/bin/env python3
"""Verify the offline bundle without ROS, network access or controller imports."""
import argparse
import hashlib
import json
from pathlib import Path
import sys


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--allow-extra', action='store_true')
    args = parser.parse_args()
    root = args.root.resolve()
    failures, expected = [], set()
    for line in (root / 'SHA256SUMS').read_text().splitlines():
        checksum, name = line.split('  ', 1)
        path = root / name
        expected.add(name)
        if not path.resolve().is_relative_to(root) or path.is_symlink():
            failures.append('Unsafe path: ' + name)
        elif not path.is_file():
            failures.append('Missing: ' + name)
        elif digest(path) != checksum:
            failures.append('Changed: ' + name)
    actual = {str(p.relative_to(root)) for p in root.rglob('*') if p.is_file() and p.name != 'SHA256SUMS'}
    if not args.allow_extra:
        failures.extend('Unexpected: ' + p for p in sorted(actual - expected))
    meta = json.loads((root / '09_PROVENANCE/package_metadata.json').read_text())
    hardware = root / '03_CONTROLLER_REPO/controllers/hardware/hnuter_external_direct_drcda_v2_hardware.py'
    if hardware.exists() and digest(hardware) != meta['hardware_entry_sha256']:
        failures.append('Hardware entry differs from recorded working-tree source')
    index = json.loads((root / '02_CONVERSATIONS/index.json').read_text())
    messages = (root / '02_CONVERSATIONS/messages.jsonl').read_text().splitlines()
    if len(messages) != index['merged_messages']:
        failures.append('Conversation index/message count mismatch')
    if failures:
        print('\n'.join(failures), file=sys.stderr)
        return 1
    print(json.dumps(dict(status='OK', verified_files=len(expected),
                          conversations=len(index['sessions']), messages=len(messages),
                          hardware_sha256=meta['hardware_entry_sha256']), ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
