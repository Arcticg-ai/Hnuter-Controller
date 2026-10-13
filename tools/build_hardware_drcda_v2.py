#!/usr/bin/env python3
"""Rebuild the standalone hardware entry without runtime repository imports."""
import argparse
import ast
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OLD = ROOT / 'controllers/hardware/hnuter_external_direct_drcda_hardware.py'
# Pin the reviewed implementation so unrelated simulation edits cannot silently
# change a hardware deployment or make a clean checkout non-reproducible.
CORE = ROOT / 'tools/templates/drcda_hardware_v2_allocator.py.in'
EXTRA = ROOT / 'tools/templates/drcda_hardware_v2.py.in'
OUTPUT = ROOT / 'controllers/hardware/hnuter_external_direct_drcda_v2_hardware.py'
CONFIG = ROOT / 'config/hardware/hnuter_drcda_v2_hardware_tuning.json'


def segment(source, name):
    node = next(n for n in ast.parse(source).body if getattr(n, 'name', '') == name)
    return '\n'.join(source.splitlines()[node.lineno - 1:node.end_lineno]) + '\n'


def render():
    old, core = OLD.read_text(), CORE.read_text()
    # Use the hardware geometry/calibration, NOT the shared model's SITL defaults.
    old_tree = ast.parse(old)
    first = next(n.lineno for n in old_tree.body if getattr(n, 'name', '') == 'DRCDAConfig')
    # These simulator-only publishers have no role on a physical vehicle.
    omitted = set()
    replacements = {}
    for node in ast.walk(old_tree):
        if isinstance(node, ast.FunctionDef) and node.name == 'workspace_root':
            replacements[node.lineno] = (
                'def workspace_root() -> Path:\n'
                '    here = Path(__file__).resolve().parent\n'
                "    if here.name == 'hardware' and here.parent.name == 'controllers':\n"
                '        return here.parent.parent\n'
                '    return Path.cwd()'
            )
            omitted.update(range(node.lineno, node.end_lineno + 1))
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == 'default_tuning_path' for t in node.targets):
            replacements[node.lineno] = (
                "        default_tuning_path = workspace_root() / 'config' / 'hardware' / self._default_tuning_filename()"
            )
            omitted.update(range(node.lineno, node.end_lineno + 1))
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Attribute) and t.attr in
                {'gz_servo0_pub', 'gz_servo1_pub', 'gz_servo2_pub', 'gz_servo3_pub'}
                for t in node.targets):
            omitted.update(range(node.lineno, node.end_lineno + 1))
    base = '\n'.join(replacements.get(i, line)
                     for i, line in enumerate(old.splitlines()[:first - 2], 1)
                     if i not in omitted or i in replacements)
    core_nodes = ast.parse(core).body
    first_constant = next(n.lineno for n in core_nodes if isinstance(n, ast.Assign))
    model = next(n for n in core_nodes if getattr(n, 'name', '') == 'HnuterWrenchModel')
    lines = core.splitlines()
    allocator = '\n'.join(lines[first_constant - 1:model.lineno - 1])
    allocator += '\n' + segment(old, 'HnuterWrenchModel')
    allocator += '\n' + '\n'.join(lines[model.end_lineno:])
    config = json.loads(CONFIG.read_text())
    inputs = (OLD, CORE, EXTRA, CONFIG)
    provenance = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in inputs}
    result = base + '\n\n' + allocator + '\n\n'
    result += segment(old, 'HnuterHardwareDRCDAController') + '\n'
    result += 'HARDWARE_V2_BUILD_INPUTS = ' + repr(provenance) + '\n'
    result += 'HARDWARE_V2_DEFAULT_CONFIG = ' + repr(config) + '\n\n'
    result += EXTRA.read_text()
    compile(result, str(OUTPUT), 'exec')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    content = render()
    if args.check:
        if not OUTPUT.exists() or OUTPUT.read_text() != content:
            raise SystemExit('Standalone output differs; rebuild with this script.')
        print('Standalone output matches its four recorded inputs.')
    else:
        OUTPUT.write_text(content)
        print(OUTPUT)


if __name__ == '__main__':
    main()
