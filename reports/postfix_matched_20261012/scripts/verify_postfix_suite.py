"""Check the complete raw archive, plot outputs, and PDF text boundaries."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET
import pandas as pd
from PIL import Image


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);root=p.parse_args().root.resolve()
    verification=json.loads((root/'verification.json').read_text())
    assert verification['complete_matrix'] and len(verification['checks'])==18
    metrics=pd.read_csv(root/'metrics.csv')
    assert len(metrics)==18 and not metrics.duplicated(['scenario','method']).any()
    for case in verification['checks']:
        folder=root/'data'/case['scenario']/'repeat_1'/case['method']
        assert digest(folder/'flight.ulg')==case['ulog_sha256']
        assert digest(folder/'effective_tuning.json')==case['tuning_sha256']
        assert case['direct_actuation_verified'] and case['unit_gain'] and not case['pure_delay']
        d=pd.read_csv(folder/'plot_data.csv',low_memory=False)
        assert len(d)>100 and not d[['ep','true_reference_so3_deg']].isna().any().any()
    manifest=json.loads((root/'run_manifest.json').read_text())
    firmware=Path(manifest['firmware']['path'])
    live_matches={name:digest(firmware/name)==sha for name,sha in manifest['firmware_sha256'].items()}
    assert all(live_matches.values()),'Firmware changed since the experiment'
    figs=root/'publication_figures';pdfs=sorted(figs.rglob('*.pdf'));assert len(pdfs)==18
    pdf_checks=[]
    for path in pdfs:
        info=subprocess.check_output(['pdfinfo',str(path)],text=True)
        pages=int(re.search(r'Pages:\s+(\d+)',info).group(1))
        assert pages==(8 if path.parent==figs else 1)
        tree=ET.fromstring(subprocess.check_output(['pdftotext','-bbox',str(path),'-']))
        words=0
        for page in tree.iter('{http://www.w3.org/1999/xhtml}page'):
            w,h=float(page.attrib['width']),float(page.attrib['height'])
            for word in page.iter('{http://www.w3.org/1999/xhtml}word'):
                a=word.attrib;words+=1
                assert float(a['xMin'])>=0 and float(a['yMin'])>=0,(path,word.text,a)
                assert float(a['xMax'])<=w+.5 and float(a['yMax'])<=h+.5,(path,word.text,a)
        assert words>20
        pdf_checks.append(dict(file=str(path.relative_to(root)),pages=pages,text_in_page=True))
    for theme in ['TRO','SR']:
        pngs=list((figs/theme).glob('*.png'));svgs=list((figs/theme).glob('*.svg'))
        assert len(pngs)==len(svgs)==8
        for path in pngs:
            with Image.open(path) as im:
                assert im.width==3240 and abs(im.info['dpi'][0]-450)<1
    testxml=ET.parse(root/'regression_tests.xml')
    suites=list(testxml.getroot().iter('testsuite'))
    assert sum(int(s.attrib.get('failures',0))+int(s.attrib.get('errors',0)) for s in suites)==0
    tests=sum(int(s.attrib['tests']) for s in suites)
    out=dict(raw_ulogs_verified=18,unchanged_firmware_hashes=live_matches,regression_tests=tests,
        pdf_count=18,figures_per_style=8,png_dpi=450,pdf_checks=pdf_checks,
        scope='Text-boundary checks supplement the separately inspected PDF raster previews.')
    (root/'artifact_verification.json').write_text(json.dumps(out,indent=2)+'\n')
    if Path(__file__).resolve()!=(root/'scripts'/Path(__file__).name).resolve():
        shutil.copy2(__file__,root/'scripts'/Path(__file__).name)
    print(json.dumps({k:v for k,v in out.items() if k not in ['pdf_checks','unchanged_firmware_hashes']},indent=2))


if __name__=='__main__':main()
