"""Reproducible guarded firmware patch, preserving originals and binary first."""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import shutil

ROOT=Path(__file__).resolve().parent
PX4=Path('/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl')


def replace(text,old,new):
    if text.count(old)!=1:raise RuntimeError('Source anchor changed: '+old[:80])
    return text.replace(old,new)


def main():
    p=argparse.ArgumentParser();p.add_argument('--apply',action='store_true');args=p.parse_args()
    paths=['src/modules/mc_rate_control/HnuterControl.hpp','src/modules/mc_rate_control/HnuterControl.cpp','msg/HnuterControlStatus.msg']
    original={name:(PX4/name).read_text() for name in paths}
    if 'JEPA_SITL_MASS' in original[paths[0]]:raise RuntimeError('Patch already applied; refusing duplicate application')
    changed=original.copy()
    h=changed[paths[0]]
    h=replace(h,'#include <uORB/topics/vehicle_control_mode.h>', '#include <uORB/topics/vehicle_control_mode.h>\n#if defined(CONFIG_ARCH_BOARD_PX4_SITL)\n#include <uORB/topics/vehicle_command.h>\n#endif')
    h=replace(h,'\tmatrix::Vector3f _velocity_integral{};', '''#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
\t// JEPA_SITL_MASS: compiled out on physical flight-controller targets.
\tuORB::Subscription _jepa_mass_command_sub{ORB_ID(vehicle_command)};
\tfloat _jepa_mass_effective{0.f};
\tfloat _jepa_mass_target{0.f};
\thrt_abstime _jepa_mass_request_time{0};
#endif
\tmatrix::Vector3f _velocity_integral{};''')
    changed[paths[0]]=h
    c=changed[paths[1]]
    c=replace(c,'void HnuterControl::reset()\n{','''void HnuterControl::reset()
{
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
\t_jepa_mass_effective = 0.f;
\t_jepa_mass_target = 0.f;
\t_jepa_mass_request_time = 0;
#endif''')
    c=replace(c,'\tconst float mass = math::max(_param_hntr_mass.get(), 0.1f);','''\tfloat mass = math::max(_param_hntr_mass.get(), 0.1f);
\tfloat jepa_mass_force_error = 0.f;
\tbool jepa_mass_updated = false;
\tbool jepa_mass_rejected = false;
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
\tif (_jepa_mass_effective < 0.1f) {
\t\t_jepa_mass_effective = mass;
\t\t_jepa_mass_target = mass;
\t}
\tmass = _jepa_mass_effective;
\tvehicle_command_s mass_request{};
\tif (_jepa_mass_command_sub.update(&mass_request)
\t    && mass_request.command == 31000 && mass_request.from_external
\t    && mass_request.target_system == 1 && mass_request.source_system == 73
\t    && fabsf(mass_request.param2 - 54123.f) < 0.1f
\t    && PX4_ISFINITE(mass_request.param1)
\t    && mass_request.param1 >= 3.5f && mass_request.param1 <= 7.f
\t    && mass_request.timestamp <= now && now - mass_request.timestamp < 500000) {
\t\t_jepa_mass_target = mass_request.param1;
\t\t_jepa_mass_request_time = now;
\t}
\t// Loss of the acceptance heartbeat freezes the current mass, no parameter write.
\tif (now - _jepa_mass_request_time > 500000) {
\t\t_jepa_mass_target = mass;
\t}
#endif''')
    c=replace(c,'\tconst Vector3f acc_unsaturated = acc_ff + velocity_p * vel_error + _velocity_integral','\tVector3f acc_unsaturated = acc_ff + velocity_p * vel_error + _velocity_integral')
    anchor='\tacc_des(2) = math::constrain(acc_des(2), -max_acc_up, max_acc_down);'
    c=replace(c,anchor,anchor+'''

#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
\t// Transform the PRE-integration state used by this cycle's force computation.
\t// Reject rather than clamp: both integral and acceleration limits must allow
\t// exactly the same desired force. Existing integration then proceeds normally.
\tconst bool mass_gate = !landed && !maybe_landed && have_local_pos
\t\t&& local_pos.xy_valid && local_pos.z_valid && local_pos.v_xy_valid && local_pos.v_z_valid
\t\t&& local_pos.timestamp <= now && now - local_pos.timestamp < 200000
\t\t&& vehicle_control_mode.flag_control_offboard_enabled
\t\t&& Dcmf{Quatf(att.q)}(2, 2) > 0.93969f
\t\t&& (acc_unsaturated - acc_des).norm() < 0.00001f
\t\t&& !tilt_suppress_active && !xy_lock_active && !takeoff_release_ramp_active;
\tif (fabsf(_jepa_mass_target - mass) > 0.00001f && mass_gate) {
\t\tconst float proposed = mass + math::constrain(_jepa_mass_target - mass, -0.3f * dt, 0.3f * dt);
\t\tconst Vector3f grav{0.f, 0.f, gravity};
\t\tconst Vector3f base_acc = acc_unsaturated - _velocity_integral;
\t\tconst Vector3f integral_proposed = (mass / proposed) * (acc_des - grav) - base_acc + grav;
\t\tconst Vector3f acc_proposed = base_acc + integral_proposed;
\t\tVector3f bounded_acc = acc_proposed;
\t\tconstrainXY(bounded_acc, max_acc_xy);
\t\tconst float proposed_up_limit = math::min(requested_max_acc_z,
\t\t\tmath::max(max_front_vertical_thrust / proposed - gravity, 0.1f));
\t\tbounded_acc(2) = math::constrain(bounded_acc(2), -proposed_up_limit, max_acc_down);
\t\tjepa_mass_force_error = Vector3f{proposed * (acc_proposed - grav) - mass * (acc_des - grav)}.norm();
\t\tif (Vector2f{integral_proposed(0), integral_proposed(1)}.norm() <= _param_hntr_vel_ilim_xy.get()
\t\t    && fabsf(integral_proposed(2)) <= _param_hntr_vel_ilim_z.get()
\t\t    && (bounded_acc - acc_proposed).norm() < 0.00001f && jepa_mass_force_error < 0.0001f) {
\t\t\t_velocity_integral = integral_proposed;
\t\t\tacc_des = acc_proposed;
\t\t\tacc_unsaturated = acc_proposed;
\t\t\tmass = proposed;
\t\t\t_jepa_mass_effective = proposed;
\t\t\tjepa_mass_updated = true;
\t\t} else {
\t\t\tjepa_mass_rejected = true;
\t\t}
\t}
#endif
''')
    c=replace(c,'\thnuter_status.active = geometric_control_active;','''\thnuter_status.active = geometric_control_active;
\thnuter_status.effective_mass = mass;
\thnuter_status.mass_target = mass;
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
\thnuter_status.mass_target = _jepa_mass_target;
#endif
\thnuter_status.mass_update_force_error = jepa_mass_force_error;
\thnuter_status.mass_updated = jepa_mass_updated;
\thnuter_status.mass_update_rejected = jepa_mass_rejected;''')
    changed[paths[1]]=c
    changed[paths[2]]+='''
# JEPA SITL diagnostics; effective mass equals HNTR_MASS on hardware builds.
float32 effective_mass
float32 mass_target
float32 mass_update_force_error
bool mass_updated
bool mass_update_rejected
'''
    patches=[]
    backup=ROOT/'firmware_original';backup.mkdir(exist_ok=True)
    for name in paths:
        p=backup/name;p.parent.mkdir(parents=True,exist_ok=True)
        if p.exists() and p.read_text()!=original[name]:raise RuntimeError('Refusing to overwrite a different backup')
        p.write_text(original[name])
        p=ROOT/'firmware_patched'/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(changed[name])
        patches.extend(difflib.unified_diff(original[name].splitlines(True),changed[name].splitlines(True),fromfile='a/'+name,tofile='b/'+name))
    (ROOT/'firmware.patch').write_text(''.join(patches))
    binary=backup/'px4_original'
    if not binary.exists():shutil.copy2(PX4/'build/px4_sitl_default/bin/px4',binary)
    (backup/'sha256.json').write_text(json.dumps({name:hashlib.sha256(original[name].encode()).hexdigest() for name in paths},indent=2)+'\n')
    if args.apply:
        for name in paths:
            if (PX4/name).read_text()!=original[name]:raise RuntimeError('Concurrent source modification')
        for name in paths:(PX4/name).write_text(changed[name])
    print('Applied' if args.apply else 'Prepared',ROOT/'firmware.patch')


if __name__=='__main__':main()
