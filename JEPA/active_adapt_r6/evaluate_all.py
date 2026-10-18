from pathlib import Path
import json,sys
from evaluate import evaluate
R=Path(__file__).resolve().parent
for name in json.loads((R/'runtime/MATRIX.json').read_text())['order']:
    if '--rebuild' in sys.argv or not (R/'results'/f'{name}_evaluation.json').exists():evaluate(name)
