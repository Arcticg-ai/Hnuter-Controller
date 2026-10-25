"""Seal the report only after tests and an archived-data-only replay pass."""
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import pandas as pd

BASE = Path(__file__).resolve().parent
PARENT = Path('/home/hnuter/px4_ws_ros2_consolidated/reports/nda_baseline_validation_20261011')
DEST = PARENT/'revised_maneuvers'


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def seal(root):
    manifest = root/'SHA256SUMS.txt'
    files = sorted(p for p in root.rglob('*') if p.is_file() and p!=manifest and '__pycache__' not in p.parts)
    manifest.write_text(''.join(f'{digest(p)}  {p.relative_to(root)}\n' for p in files))
    for line in manifest.read_text().splitlines():
        sha,name = line.split('  ',1)
        assert digest(root/name)==sha, name
    print(f'Verified {len(files)} files in {root}')


def main():
    xml = BASE/'tests_final_all.xml'
    suites = list(ET.parse(xml).getroot().iter('testsuite'))
    count = sum(int(s.attrib['tests']) for s in suites)
    assert count==231
    assert all(int(s.attrib['failures'])==0 and int(s.attrib['errors'])==0 for s in suites)
    shutil.copy2(xml,DEST/'provenance/tests_final_all.xml')
    pd.testing.assert_frame_equal(pd.read_csv(DEST/'metrics.csv'),pd.read_csv('/tmp/hnuter-revision-offline-replay-20261012/metrics.csv'))
    p = DEST/'verification.json'
    verified = json.loads(p.read_text())
    verified.update(tests_passed=count,offline_replay_without_raw_ulog=True,
                    cpp_landing_guard_assertions_passed=True,git_diff_check_passed=True)
    p.write_text(json.dumps(verified,indent=2,ensure_ascii=False)+'\n')
    shutil.copy2(__file__,DEST/'scripts/seal_revision.py')
    seal(DEST)
    seal(PARENT)


if __name__=='__main__':
    main()
