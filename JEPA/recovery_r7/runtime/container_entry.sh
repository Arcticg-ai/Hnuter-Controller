#!/bin/bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /opt/ws/install/setup.bash
export PATH=/opt/venv/bin:$PATH
export PYTHONPATH=/work/fixed:/work:/opt/hnuter:/opt/ws:/opt/ws/JEPA/revision2:/opt/ws/JEPA/revision3:/opt/ws/JEPA/revision3/articulated:${PYTHONPATH:-}
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 PYTHONDONTWRITEBYTECODE=1
cd /work
exec python "$@"
