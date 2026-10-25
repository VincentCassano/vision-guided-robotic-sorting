"""File provenance for empty-withdrawal repair; normal v2 recipe/guards unchanged."""
import hashlib
from pathlib import Path
from visual_pick_sort_v2_source import accepted_pregrasp,PREPARE,RECIPE,grip_recipe_proof
ROOT=Path(__file__).resolve().parents[1]
MANIFEST=ROOT/'deployment/visual-pick-sort-v3-files.sha256'

def verify_delivery():
    result={}
    for line in MANIFEST.read_text(encoding='ascii').splitlines():
        expected,name=line.split('  ',1);path=(ROOT/name).resolve()
        if (ROOT not in path.parents or name in result or len(expected)!=64
                or hashlib.sha256(path.read_bytes()).hexdigest()!=expected):
            raise ValueError('Delivery file changed: '+name)
        result[name]=expected
    if not result:raise ValueError('Empty delivery inventory')
    return result

