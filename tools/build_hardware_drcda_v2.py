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
    additions = {}
    trim_sites = 0
    wrench_sites = 0
    for node in ast.walk(old_tree):
        if (isinstance(node, ast.Constant) and isinstance(node.value, str)
                and node.value.startswith('HARDWARE MODE:')):
            # The old entry's always-on heartbeat instructions do not apply to
            # v2's monitor default and must not suggest an unvalidated flight.
            replacements[node.lineno] = ' ' * node.col_offset + repr(
                'HARDWARE V2: monitor-only by default; no heartbeat or actuator output in monitor mode. '
                'No automatic Arm/Disarm/Offboard; critical faults may request Position. Check v2 output status; '
                'prop-off/bench validation is required before enabling takeover.')
            omitted.update(range(node.lineno, node.end_lineno + 1))
        if (isinstance(node, ast.Assign) and node.lineno < first
                and any(isinstance(t, ast.Name) and t.id == 'tau_c' for t in node.targets)
                and isinstance(node.value, ast.BinOp)):
            indent = ' ' * node.col_offset
            additions[node.end_lineno] = indent + 'tau_c = self._add_firmware_attitude_trim(tau_c)'
            trim_sites += 1
        if isinstance(node, ast.FunctionDef) and node.name == '_apply_firmware_tail_pitch_bias':
            # V2 adds gravity and bias in physical FRD torque before saturation.
            # Keep this legacy call site solely for the allocator sign/scale.
            replacements[node.lineno] = (
                '    def _apply_firmware_tail_pitch_bias(\n'
                '        allocator_pitch_torque, max_tail_thrust, tail_arm,\n'
                '        pitch_torque_bias, tail_torque_sign,\n'
                '    ):\n'
                '        scale = max(float(max_tail_thrust) * float(tail_arm), 1e-8)\n'
                '        direction = -1.0 if tail_torque_sign < 0 else 1.0\n'
                '        return direction * float(np.clip(allocator_pitch_torque, -scale, scale))'
            )
            omitted.update(range(node.lineno, node.end_lineno + 1))
        if (isinstance(node, ast.Assign) and isinstance(node.value, ast.Call)
                and isinstance(node.value.func, ast.Attribute)
                and node.value.func.attr == '_apply_firmware_tail_pitch_bias'):
            additions[node.end_lineno] = (' ' * node.col_offset
                + 'W = self._apply_hardware_wrench_handover(W)')
            wrench_sites += 1
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
    if trim_sites != 2:
        raise ValueError('Expected exactly two attitude torque construction sites')
    if wrench_sites != 1:
        raise ValueError('Expected exactly one hardware wrench handover site')
    base = '\n'.join(replacements.get(i, line) + ('\n' + additions[i] if i in additions else '')
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
    provenance = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in inputs}
    result = base + '\n\n' + allocator + '\n\n'
    hardware_controller = segment(old, 'HnuterHardwareDRCDAController')
    if hardware_controller.count('command = self.drcda.command') != 1:
        raise ValueError('Expected exactly one held allocation publication site')
    hardware_controller = hardware_controller.replace(
        'command = self.drcda.command', 'command = self.drcda.held_solution')
    result += hardware_controller + '\n'
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
        OUTPUT.write_text(content, encoding='utf-8', newline='\n')
        print(OUTPUT)


if __name__ == '__main__':
    main()
