#!/usr/bin/env python3
"""Create a new v2 cascade profile while retaining existing aircraft calibration."""
import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT = ROOT / 'config/hardware/hnuter_drcda_v2_hardware_tuning.json'
CHANGED = ('rc_attitude_rate_deg_s', 'gamepad_max_acc_body_xy_mps2',
           'gamepad_filter_tau_body_xy_s', 'gamepad_filter_tau_s')


def migrate(source, defaults):
    data = dict(source.get('tuning', source))
    required = {k for k in defaults if not k.startswith('cascade_') and k not in CHANGED}
    if required - data.keys():
        raise ValueError('incomplete source tuning: ' + ', '.join(sorted(required-data.keys())))
    for key in defaults:
        if key.startswith('cascade_') or key in CHANGED:
            data[key] = defaults[key]
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path, help='Existing tuning JSON or runtime snapshot')
    parser.add_argument('output', type=Path, help='New file; never overwrite old tuning')
    args = parser.parse_args()
    data = migrate(json.loads(args.source.read_text()), json.loads(DEFAULT.read_text()))
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(data, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(args.output)


if __name__ == '__main__':
    main()
