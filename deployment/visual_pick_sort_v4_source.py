"""File provenance; one empty opposition withdrawal, then a new hand-pose candidate."""
import hashlib
from pathlib import Path
from visual_pick_sort_source import accepted_pregrasp
from visual_pick_sort_v2_source import grip_recipe_proof as previous_grip_proof
ROOT=Path(__file__).resolve().parents[1]
MANIFEST=ROOT/'deployment/visual-pick-sort-v4-files.sha256'
OLD_PREPARE=[8000,8000,8000,8000,65535,65056]
PREPARE=[8000,8000,8000,8000,65535,64556]
RECIPE=[26500,30500,27000,19000,65535,64556]

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
    old=previous_grip_proof()
    if old['recipe']!=[26500,30500,27000,19000,65535,65056]:raise ValueError('v2 recipe changed')
    return dict(previous_recipe_proof=old,recipe=RECIPE[:],
        only_new_change='opposition target65056 to64556 via one explicit empty-hand withdrawal',
        retreat_logic_counts=500,calibration_compensation=False,
        source_raw_archive_sha256='eb2f1aa8fbdd886cac14bc873308be4a818ac3a77d135e94cf2b9f1019fc784e',
        current_fresh_feedback_and_operator_new_noncontact_grip_confirmation_required=True,
        new_opposition_pose_or_recipe_real_device_accepted=False,
        old_fault_arm_or_hand_snapshot_is_not_new_startup_reference=True)
