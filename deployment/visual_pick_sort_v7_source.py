"""File provenance; one empty opposition withdrawal, then a new hand-pose candidate."""
import hashlib
from pathlib import Path
from visual_pick_sort_source import accepted_pregrasp as old_accepted_pregrasp
from visual_pick_sort_v2_source import grip_recipe_proof as previous_grip_proof
ROOT=Path(__file__).resolve().parents[1]
MANIFEST=ROOT/'deployment/visual-pick-sort-v7-files.sha256'
OLD_PREPARE=[8000,8000,8000,8000,65535,65056]
PREPARE=[8000,8000,8000,8000,65535,64556]
RECIPE=[27500,31500,27000,20500,65535,64556]

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
        only_new_change='relative to v6 ring19000->20500; thumb27500/index31500/middle27000/little65535/opposition64556 unchanged; one candidate only',
        retreat_logic_counts=500,calibration_compensation=False,
        source_raw_archive_sha256='876d369d4bc6656d31120caea7e6f272d9c53b5d404a935699f53f8425d404d7',
        current_fresh_feedback_and_operator_new_noncontact_grip_confirmation_required=True,
        new_opposition_pose_or_recipe_real_device_accepted=False,
        old_fault_arm_or_hand_snapshot_is_not_new_startup_reference=True)


def accepted_pregrasp(path,expected_sha=None):
    import json,zipfile
    with zipfile.ZipFile(path) as z:
        root=z.namelist()[0].split('/')[0]+'/'
        parent=json.loads(z.read(root+'result.json'))
    if parent.get('deployed_source_sha256')==verify_delivery():
        from visual_pick_sort_v7_pregrasp_source import accepted_pregrasp as current
        return current(path,expected_sha)
    return old_accepted_pregrasp(path,expected_sha)


def back40_offset():
    import json,math
    raw=(ROOT/'deployment/camera_position_candidate_225543.json').read_bytes()
    if hashlib.sha256(raw).hexdigest()!='3edef2810580a1de04db14bafb3aabc19c8fd6fcac903641b83a1d0a1a9380cb':
        raise ValueError('Fixed head camera direction source changed')
    matrix=json.loads(raw)['world_from_camera'];forward=[matrix[0][2],matrix[1][2]]
    length=math.hypot(*forward)
    if length<.1:raise ValueError('Fixed head forward projection ill-conditioned')
    return [-.040*v/length for v in forward]+[0.]


def qualified_v6_bin(path):
    import json,zipfile
    raw=Path(path).read_bytes()
    if hashlib.sha256(raw).hexdigest()!='876d369d4bc6656d31120caea7e6f272d9c53b5d404a935699f53f8425d404d7':
        raise ValueError('Expected original230421 v6 archive')
    with zipfile.ZipFile(path) as z:
        names=z.namelist();root=names[0].split('/')[0]+'/'
        get=lambda n:json.loads(z.read(root+n))
        hashes=get('source_hashes.json');parent=get('result.json');worker=get('worker_result.json')
        if (len(names)!=len(set(names)) or z.testzip()
                or set(hashes)!={n[len(root):] for n in names if n!=root+'source_hashes.json'}
                or any(hashlib.sha256(z.read(root+n)).hexdigest()!=h for n,h in hashes.items())
                or parent['worker']!=worker or worker['cleanup_errors']
                or worker['arm_actuator_writes']!=4 or worker['hand_register_writes']!=1
                or worker['controller_configuration_writes']!=0
                or [e['error'] for e in worker['exceptions']]!=["NameError(\"name 'normal_drop_values' is not defined\")"]):
            raise ValueError('Original230421 evidence differs')
        segments=worker['arm_segments']
        if ([s['phase'] for s in segments]!=['lifting_loaded_140mm','school_high_transit','school_restart_transit','school_black_drop']
                or any(not s['notification']['acknowledged'] or not s['notification']['event_seen'] for s in segments)):
            raise ValueError('Completed actual bin segment required')
        arm=segments[-1]['endpoint']
        if arm['arm_error'] or arm['system_error']:raise ValueError('Completed endpoint not healthy')
        for n,h in parent['delivery_sha256'].items():
            if hashlib.sha256((ROOT/n).read_bytes()).hexdigest()!=h:raise ValueError('Original v6 source changed')
        return dict(arm=arm,source_sha256=hashlib.sha256(raw).hexdigest(),
            source_role='completed_bin_segment_not_fault_last_arm_state',
            physical_pickup_or_sort_success=False,old_fault_state_used=False,fresh_current_match_required=True)
