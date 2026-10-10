"""Read-only diagnostics. Does not arm, start PX4, or contact any endpoint."""
import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parent
PX4 = Path('/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl')


def command(args, cwd=None):
    try:
        r = subprocess.run(args, cwd=cwd, capture_output=True, text=True, timeout=15)
        return dict(argv=args, returncode=r.returncode, stdout=r.stdout, stderr=r.stderr)
    except (OSError, subprocess.TimeoutExpired) as e:
        return dict(argv=args, error=repr(e))


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):
            h.update(block)
    return h.hexdigest()


def main():
    stamp = datetime.datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S')
    out = ROOT/'runs'/('preflight_'+stamp)
    out.mkdir(parents=True, exist_ok=False)
    report = dict(timezone='Asia/Shanghai', timestamp=stamp, sockets={}, repos={},
                  isolation_proposal=dict(ROS_DOMAIN_ID=73, ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                                          PX4_UXRCE_DDS_PORT=28888, PX4_UXRCE_DDS_NS='jepa_sitl',
                                          GZ_PARTITION='jepa_mass_cal_unique_per_run', GZ_IP='127.0.0.1'),
                  scope='Diagnostics only. No PX4, simulator, controller or hardware launched.')
    for label, kind in [('udp', socket.SOCK_DGRAM),('tcp',socket.SOCK_STREAM)]:
        try:
            with socket.socket(socket.AF_INET, kind) as s:
                s.bind(('127.0.0.1',0))
                report['sockets'][label] = dict(ok=True, local_address=s.getsockname())
        except OSError as exc:
            report['sockets'][label] = dict(ok=False, error=repr(exc), errno=exc.errno)
    for name, path in [('controller',ROOT.parent),('px4',PX4)]:
        report['repos'][name] = {key:command(['git',*args],path) for key,args in
            [('branch',['branch','--show-current']),('head',['rev-parse','HEAD']),('status',['status','--short'])]}
        diff = command(['git','diff','--binary'],path)
        (out/(name+'_existing_changes.diff')).write_text(diff.get('stdout',''))
    tracked_hashes = {}
    for line in command(['git','diff','--name-only'],ROOT.parent).get('stdout','').splitlines():
        p = ROOT.parent/line
        if p.is_file():
            tracked_hashes[line] = sha(p)
    report['preexisting_modified_file_sha256'] = tracked_hashes
    paths = [PX4/'build/px4_sitl_default/bin/px4', PX4/'Tools/simulation/gz/models/hnuter/model.sdf',
             ROOT.parent/'hnuter_external_controller_px4_position.py']
    report['input_sha256'] = {str(p):sha(p) for p in paths if p.is_file()}
    report['build_version_header'] = (PX4/'build/px4_sitl_default/src/lib/version/build_git_version.h').read_text()
    report['executables'] = {x:shutil.which(x) for x in ['MicroXRCEAgent','gz','python3','make']}
    report['environment'] = {k:os.environ.get(k) for k in ['ROS_DISTRO','ROS_DOMAIN_ID','RMW_IMPLEMENTATION','GZ_PARTITION','PX4_SIM_MODEL']}
    report['gz_version'] = command(['gz','sim','--versions'])
    report['ros_import'] = command(['bash','-c',"source /opt/ros/jazzy/setup.bash\nsource /home/hnuter/px4_ws_ros2/install/setup.bash\npython3 -c 'import rclpy; from px4_msgs.msg import VehicleLocalPosition; print(\"ROS imports OK\")'"])
    # Do not confuse UNIX mode bits with sandbox write permission.
    report['filesystem_policy'] = dict(jepa='Writable per session policy', px4='Read-only per session policy; no write attempted')
    report['status'] = 'BLOCKED_LOCAL_SOCKETS' if not all(v['ok'] for v in report['sockets'].values()) else 'SOCKET_PROBE_OK_NOT_FLIGHT_READY'
    (out/'preflight.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(status=report['status'],output=str(out),sockets=report['sockets']),indent=2))
    return 2 if report['status'].startswith('BLOCKED') else 0


if __name__ == '__main__':
    raise SystemExit(main())
