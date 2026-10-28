"""Archive measured simulation curves and frozen inputs, excluding aircraft logs and firmware binaries."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil

from tools.experiments.hardware_repair_core import sha256, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    evidence = args.output / 'evidence'
    evidence.mkdir(exist_ok=True)
    inventory = []
    for path in sorted(args.root.glob('*/manifest.json')):
        campaign = path.parent
        dest = evidence / campaign.name
        dest.mkdir(exist_ok=True)
        shutil.copy2(path, dest/'manifest.json')
        shutil.copytree(campaign/'inputs', dest/'inputs', dirs_exist_ok=True,
                        ignore=shutil.ignore_patterns('__pycache__'))
        for case in sorted(campaign.glob('runs/**/capability.json')):
            run = case.parent
            run_dest = dest / run.relative_to(campaign)
            run_dest.mkdir(parents=True, exist_ok=True)
            for name in ('result.json', 'completion.json', 'operator_events.jsonl', 'capability.json', 'isolation.json'):
                if (run/name).exists():
                    shutil.copy2(run/name, run_dest/name)
            samples = run/'samples.jsonl'
            if not samples.exists():
                samples = run/'samples.jsonl.gz'
            for source in [samples, *run.glob('logs/external_control/hardware/drcda_v2/*.csv*')]:
                if source.exists():
                    if source.suffix == '.gz':
                        target = run_dest/source.name
                        shutil.copy2(source, target)
                        with gzip.open(source, 'rb') as handle:
                            digest = hashlib.sha256(handle.read()).hexdigest()
                        inventory.append(dict(path=str(target.relative_to(args.output)),
                                              uncompressed_sha256=digest, compressed_sha256=sha256(target)))
                        continue
                    target = run_dest/(source.name+'.gz')
                    with source.open('rb') as inp, target.open('wb') as raw:
                        with gzip.GzipFile(filename='', mode='wb', fileobj=raw, mtime=0) as out:
                            shutil.copyfileobj(inp, out)
                    inventory.append(dict(path=str(target.relative_to(args.output)),
                                          uncompressed_sha256=sha256(source), compressed_sha256=sha256(target)))
    shutil.copytree(args.root/'figures', args.output/'figures', dirs_exist_ok=True)
    write_json(args.output/'inventory.json', inventory)
    print(f'Archived {len(inventory)} simulation trace files; aircraft logs and binaries excluded.')


if __name__ == '__main__':
    main()
