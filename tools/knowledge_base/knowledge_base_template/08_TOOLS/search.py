#!/usr/bin/env python3
"""Literal, case-insensitive search of knowledge text, without indexing big logs."""
import argparse
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('query')
    parser.add_argument('--limit', type=int, default=60)
    parser.add_argument('--scope', choices=['all', 'context', 'conversations', 'code', 'reports'], default='all')
    args = parser.parse_args()
    if not args.query or args.limit < 1:
        parser.error('query must be nonempty and limit must be positive')
    root = Path(__file__).resolve().parents[1]
    count = 0
    groups = dict(context=['01_CONTEXT'], conversations=['02_CONVERSATIONS'],
                  code=['03_CONTROLLER_REPO/controllers', '03_CONTROLLER_REPO/tools'],
                  reports=['03_CONTROLLER_REPO/docs', '03_CONTROLLER_REPO/reports',
                           '05_HARDWARE_EVIDENCE', '10_LEGACY_LOCAL'])
    directories = sum(groups.values(), []) if args.scope == 'all' else groups[args.scope]
    for directory in directories:
        for path in sorted((root / directory).rglob('*')):
            if path.suffix not in {'.md', '.txt', '.py'} or path.stat().st_size > 16 * 1024 * 1024:
                continue
            for line_num, line in enumerate(path.read_text(errors='replace').splitlines(), 1):
                if args.query.casefold() in line.casefold():
                    print(f'{path.relative_to(root)}:{line_num}: {line[:600]}')
                    count += 1
                    if count >= args.limit:
                        return
    if count == 0:
        print('No matches.')


if __name__ == '__main__':
    main()
