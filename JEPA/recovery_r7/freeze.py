from pathlib import Path
import hashlib,json
R=Path(__file__).resolve().parent
files=[p for p in R.rglob('*') if p.is_file() and '__pycache__' not in p.parts and ('results' not in p.relative_to(R).parts or p.parent==R/'results/models' or p.name=='model.sdf' and 'models' in p.relative_to(R).parts and 'runs' not in p.relative_to(R).parts or p in [R/'runtime/results/experiment/config.json',R/'runtime/results/experiment/cases.json']) and p.name!='FROZEN_SOURCES.json']
(R/'runtime/FROZEN_SOURCES.json').write_text(json.dumps({str(p.relative_to(R)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)},indent=2))
