"""Creates only simulation SDFs; these parameters never enter online.py."""
from pathlib import Path
import json,hashlib,copy,xml.etree.ElementTree as ET
R=Path(__file__).resolve().parent;runtime=R/'runtime';out=runtime/'results/experiment'
def main():
    source=R.parent/'method_audit_r4/collection/results/experiment/models/s0_00_nominal/model.sdf'
    if not source.exists():source=Path('/home/hnuter/Documents/Codex/2026-10-13/task/jepa-gazebo-r6-worktree/JEPA/method_audit_r4/collection/results/experiment/models/s0_00_nominal/model.sdf')
    tree=ET.parse(source);cases=[]
    for family in ['mass','com']:
        for seed in [0,2]:
            for excitation in ['active','conventional']:
                name=f's{seed}_00_{family}_{excitation}';root=copy.deepcopy(tree.getroot());link=root.find("model/link[@name='adaptation_payload']");mass=1.05 if family=='mass' else .9;offset=-.025 if family=='com' else 0.;pose=list(map(float,link.findtext('pose').split()));pose[1]+=offset;link.find('pose').text=' '.join(map(str,pose));link.find('inertial/mass').text=str(mass)
                size=[.1,.08,.04];a,b,c=size
                for k,v in zip(['ixx','iyy','izz'],[mass*(b*b+c*c)/12,mass*(a*a+c*c)/12,mass*(a*a+b*b)/12]):link.find('inertial/inertia/'+k).text=str(v)
                folder=out/'models'/name;folder.mkdir(parents=True,exist_ok=False);file=folder/'model.sdf';ET.ElementTree(root).write(file,encoding='utf-8',xml_declaration=True)
                cases.append(dict(id=name,seed=seed,excitation=excitation,split='sealed_test',family=family,model='/work/results/experiment/models/'+name+'/model.sdf',truth=dict(mass=4.49999998+mass,payload_mass=mass,payload_offset_y=offset,payload_size=size),target=None,disturbance=dict(start=50,end=58,force_y=0),model_sha256=hashlib.sha256(file.read_bytes()).hexdigest()))
    (out/'cases.json').write_text(json.dumps(cases,indent=2));(out/'config.json').write_text(json.dumps(dict(noise='native',controller_mass_kg=5.4),indent=2));(runtime/'MATRIX.json').write_text(json.dumps(dict(order=[c['id'] for c in cases],cases=cases,budget=dict(per_run_wall_s=180,flight_wall_s=1440,raw_gib=4),control_enabled=False),indent=2))
if __name__=='__main__':main()
