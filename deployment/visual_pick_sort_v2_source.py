"""File provenance only; lower two closure caps, never relax a protection gate."""
import hashlib
from pathlib import Path
from visual_pick_sort_source import accepted_pregrasp,PREPARE,grip_recipe_proof as historical_grip_proof
ROOT=Path(__file__).resolve().parents[1]
MANIFEST=ROOT/'deployment/visual-pick-sort-v2-files.sha256'
RECIPE=[26500,30500,27000,19000,65535,65056]

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

def grip_recipe_proof():
    old=historical_grip_proof()
    if old['recipe']!=[26500,30500,29500,22000,65535,65056]:
        raise ValueError('Historical recipe changed')
    return dict(historical_proof=old,recipe=RECIPE[:],
        unchanged_channels=[0,1,4,5],reduced_channels=[2,3],
        source_failed_run_sha256='2abff1627f767e7d564e5fc0fbaf7f9960009dfce3c783d2080a9e0f1bb4aa89',
        observed_fault_positions=[26504,30551,27794,19941,65535,65127],
        observed_fault_status=[2,5,3,3,5,5],
        reason='lower middle/ring targets below observed blocked positions; not a cleared fault or grip proof',
        new_recipe_real_device_accepted=False,old_fault_arm_state_used=False)
