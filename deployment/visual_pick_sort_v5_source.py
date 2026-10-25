"""File provenance; one empty opposition withdrawal, then a new hand-pose candidate."""
import hashlib
from pathlib import Path
from visual_pick_sort_source import accepted_pregrasp
from visual_pick_sort_v2_source import grip_recipe_proof as previous_grip_proof
ROOT=Path(__file__).resolve().parents[1]
MANIFEST=ROOT/'deployment/visual-pick-sort-v5-files.sha256'
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
        only_new_change='grip1135x4 instead of1135x6; retain64556 opposition and existing flex caps; supported four-flex recovery separately authorized',
        retreat_logic_counts=500,calibration_compensation=False,
        source_raw_archive_sha256='29a9a16a351444e011208962bcbc532d8454ddebb788fc50fe2f2afdf2059ba4',
        current_fresh_feedback_and_operator_new_noncontact_grip_confirmation_required=True,
        new_opposition_pose_or_recipe_real_device_accepted=False,
        old_fault_arm_or_hand_snapshot_is_not_new_startup_reference=True)
