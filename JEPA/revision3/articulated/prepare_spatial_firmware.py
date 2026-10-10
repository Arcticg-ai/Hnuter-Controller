"""Apply the reviewed spatial patch to the protected c0d57b04 source baseline.

Run only in the authorized PX4 experimental checkout after backing it up.
Fails on any unexpected content; does not reset, stash, change branch, or build.
"""
from pathlib import Path
import subprocess
P=Path('/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl')
patch=Path(__file__).resolve().parent/'firmware_spatial.patch'
subprocess.run(['git','-C',str(P),'apply','--check',str(patch)],check=True)
subprocess.run(['git','-C',str(P),'apply',str(patch)],check=True)
