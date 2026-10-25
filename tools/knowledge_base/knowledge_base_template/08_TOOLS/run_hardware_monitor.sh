#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
work="${DRCDA_WORKING_COPY:-$root/03_CONTROLLER_REPO}"
set +u
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
if [[ -n "${PX4_MSGS_SETUP:-}" ]]; then
    source "$PX4_MSGS_SETUP"
fi
set -u
python="${DRCDA_PYTHON:-/usr/bin/python3}"
"$python" - <<'PY'
import numpy
import rclpy
from px4_msgs.msg import VehicleStatus, OffboardControlMode
assert VehicleStatus.MESSAGE_VERSION == 1, 'Wrong px4_msgs overlay: expected VehicleStatus v1'
assert hasattr(OffboardControlMode(), 'direct_actuator'), 'Missing direct actuator interface'
print('Offline import/interface check passed; this does not verify live flight-controller telemetry.')
PY
if [[ "${1:-}" == "--check" ]]; then
    exit 0
fi
if [[ $# -gt 0 ]]; then
    printf '%s\n' 'Only --check is supported; this launcher never enables output.' >&2
    exit 2
fi
cd "$work"
export HNUTER_HARDWARE_DRCDA_ENABLE=0
export HNUTER_HARDWARE_TASKS=0
export HNUTER_TUNING_FILE="$work/config/hardware/hnuter_drcda_v2_hardware_tuning.json"
export HNUTER_LOG_DIR="${HNUTER_LOG_DIR:-$HOME/hnuter_logs/kb_monitor_$(date +%Y%m%d_%H%M%S)}"
printf 'Monitor only, no heartbeat/actuator output. Logs: %s\n' "$HNUTER_LOG_DIR"
exec "$python" controllers/hardware/hnuter_external_direct_drcda_v2_hardware.py
