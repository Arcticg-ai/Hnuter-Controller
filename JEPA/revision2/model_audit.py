"""Read-only SDF audit and separate payload model generation (zero joint pose).

No runtime estimator imports this module. Mass truth stays in experiment setup.
Supports the frame graph used by this checkout; unsupported SDF fails closed.
"""
import argparse
import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

PX4 = Path('/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl')


def pose(element):
    t = np.eye(4)
    if element is None:
        return t
    if element.get('degrees') or element.get('rotation_format'):
        raise ValueError('Unsupported pose convention')
    x, y, z, r, p, yaw = map(float, element.text.split())
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(yaw), np.sin(yaw)
    t[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                 [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    t[:3, 3] = [x, y, z]
    return t


def audit(model):
    if model.find('include') is not None or model.find('model') is not None:
        raise ValueError('Nested/included models require explicit resolution')
    frames = {'__model__': np.eye(4)}
    entities = {e.get('name'): e for e in model if e.tag in ('link', 'joint', 'frame')}

    def transform(name, visiting=()):
        if name in frames:
            return frames[name]
        if name in visiting:
            raise ValueError('Frame cycle')
        e = entities[name]
        pe = e.find('pose')
        default = e.findtext('child') if e.tag == 'joint' else (e.get('attached_to', '__model__') if e.tag == 'frame' else '__model__')
        relative = pe.get('relative_to', default) if pe is not None else default
        frames[name] = transform(relative, visiting + (name,)) @ pose(pe)
        return frames[name]

    rows = []
    for link in model.findall('link'):
        inertial = link.find('inertial')
        if inertial is None or inertial.get('auto') == 'true':
            raise ValueError('Explicit inertial required: ' + link.get('name'))
        ip = inertial.find('pose')
        if ip is not None and ip.get('relative_to'):
            raise ValueError('Inertial relative_to not supported')
        t = transform(link.get('name')) @ pose(ip)
        mass = float(inertial.findtext('mass'))
        a, b, c, d, e, f = [float(inertial.findtext('inertia/' + key, '0')) for key in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')]
        inertia = np.array([[a, d, e], [d, b, f], [e, f, c]])
        eig = np.linalg.eigvalsh(inertia)
        if mass <= 0 or min(eig) <= 0 or eig[-1] > sum(eig[:2]) + 1e-10:
            raise ValueError('Nonphysical inertial: ' + link.get('name'))
        inertia = t[:3, :3] @ inertia @ t[:3, :3].T
        rows.append(dict(link=link.get('name'), mass_kg=mass, com_model_m=t[:3, 3].tolist(), inertia_com_model_kg_m2=inertia.tolist()))
    total = sum(r['mass_kg'] for r in rows)
    com = sum(r['mass_kg'] * np.array(r['com_model_m']) for r in rows) / total
    inertia = np.zeros((3, 3))
    for row in rows:
        delta = np.array(row['com_model_m']) - com
        inertia += np.array(row['inertia_com_model_kg_m2']) + row['mass_kg'] * (np.dot(delta, delta)*np.eye(3) - np.outer(delta, delta))
    return dict(configuration='SDF reference pose; all joints at zero, model-local axes',
                total_mass_kg=total, com_model_m=com.tolist(), inertia_com_model_kg_m2=inertia.tolist(),
                principal_inertias_kg_m2=np.linalg.eigvalsh(inertia).tolist(), links=rows)


def generate(source, out, payload):
    tree = ET.parse(source)
    original = tree.getroot().find('model')
    before = audit(original)
    model = copy.deepcopy(original)
    name = 'hnuter_mass_cal_payload'
    model.set('name', name)
    com = before['com_model_m']
    # A compact 0.12 x 0.12 x 0.08 m box, rigidly mounted at nominal CoM.
    inertia = payload/12 * np.array([.12**2+.08**2, .12**2+.08**2, 2*.12**2])
    link = ET.SubElement(model, 'link', name='calibration_payload')
    ET.SubElement(link, 'pose').text = ' '.join(map(str, com)) + ' 0 0 0'
    ii = ET.SubElement(link, 'inertial')
    ET.SubElement(ii, 'mass').text = str(payload)
    tensor = ET.SubElement(ii, 'inertia')
    for key, value in zip(('ixx','iyy','izz','ixy','ixz','iyz'), [*inertia,0,0,0]):
        ET.SubElement(tensor, key).text = str(value)
    # Payload is internal to the existing body collision; no duplicate collider.
    joint = ET.SubElement(model, 'joint', name='payload_mount', type='fixed')
    ET.SubElement(joint, 'parent').text = 'base_link'
    ET.SubElement(joint, 'child').text = 'calibration_payload'
    for uri in model.findall('.//mesh/uri'):
        if not uri.text.startswith(('model://','file://','http://','https://','/')):
            uri.text = str((source.parent / uri.text).resolve())
    for link in model.findall('link'):
        wind = link.find('enable_wind')
        if wind is None:
            wind = ET.SubElement(link, 'enable_wind')
        wind.text = 'false'
    out.mkdir(parents=True, exist_ok=True)
    after = audit(model)
    for mode in ('native_noise', 'zero_sensor_noise'):
        variant = copy.deepcopy(model)
        if mode == 'zero_sensor_noise':
            for noise in variant.findall('.//sensor//noise'):
                for child in noise:
                    if child.tag in ('mean','stddev','bias_mean','bias_stddev','dynamic_bias_stddev'):
                        child.text = '0'
        folder = out / mode / name
        folder.mkdir(parents=True, exist_ok=True)
        root = ET.Element('sdf', version='1.11')
        root.append(variant)
        ET.indent(root)
        ET.ElementTree(root).write(folder/'model.sdf', encoding='utf-8', xml_declaration=True)
        (folder/'model.config').write_text(f'<model><name>{name}</name><version>1</version><sdf version="1.11">model.sdf</sdf></model>\n')
    report = dict(source=str(source), nominal=before, loaded=after,
                  payload=dict(mass_kg=payload, box_size_m=[.12,.12,.08], inertia_kg_m2=inertia.tolist()),
                  caveats=['Zero-joint assembly CoM; CoM moves with articulation.',
                           'Generated models have not been loaded in Gazebo.',
                           'zero_sensor_noise removes model sensor noise only; world/transport still require verification.'])
    (out/'mass_audit.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--source', type=Path, default=PX4/'Tools/simulation/gz/models/hnuter/model.sdf')
    p.add_argument('--out', type=Path, default=Path(__file__).parent/'generated')
    p.add_argument('--payload', type=float, default=.9)
    args = p.parse_args()
    if not np.isfinite(args.payload) or args.payload <= 0:
        p.error('payload must be finite and positive')
    result = generate(args.source, args.out, args.payload)
    print(json.dumps({k: result[k]['total_mass_kg'] for k in ('nominal','loaded')}, indent=2))
