"""Optional dependency rebuild into a NEW directory; never overwrites flown binaries."""
from pathlib import Path
import argparse,subprocess,shlex,json,hashlib,ctypes
p=argparse.ArgumentParser();p.add_argument('out',type=Path);a=p.parse_args();a.out.mkdir(exist_ok=False,parents=True);r=Path(__file__).resolve().parent;runtime=r.parent/'runtime';records=[]
for src,name,packages,shared in [(r/'AuditedJointPositionController.cc','libAuditedJointPositionController.so',['gz-sim8','gz-transport13'],True),(r/'BoundedForce.cc','libJepaBoundedForce.so',['gz-sim8','gz-plugin2'],True),(r/'ClockAudit.cc','libJepaClockAudit.so',['gz-sim8','gz-plugin2'],True),(runtime/'TruthRecorder.cc','libJepaTruthRecorder.so',['gz-sim8','gz-plugin2'],True),(runtime/'seeded_server.cc','seeded_server',['gz-sim8','gz-math7','gz-common5'],False)]:
    flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs',*packages],text=True));target=a.out/name;cmd=['c++','-O2','-std=c++17']+(['-shared','-fPIC'] if shared else [])+[str(src),'-o',str(target),*flags];subprocess.run(cmd,check=True)
    if shared:ctypes.CDLL(str(target.resolve()))
    else:
        for seed in [0,1,2,3,4]:subprocess.run([str(target),'--probe','fixed',str(seed)],check=True)
    records.append(dict(name=name,sha256=hashlib.sha256(target.read_bytes()).hexdigest(),compiled_and_loaded=shared,source=str(src)))
(a.out/'build_receipt.json').write_text(json.dumps(dict(files=records,used_in_R6_flights=False,byte_identical_to_original_not_required=True),indent=2));print('Five dependencies rebuilt and load/probe checked; original flight binaries untouched.')
