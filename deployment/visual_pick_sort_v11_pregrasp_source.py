"""Accepted visual pregrasp and historical healthy grip receipt; no device imports."""
import hashlib
import io
import json
from pathlib import Path
import zipfile

ROOT=Path(__file__).resolve().parents[1]
MANIFEST=ROOT/'deployment/visual-pick-sort-v11-files.sha256'
RECIPE=[26500,30500,29500,22000,65535,65056]
PREPARE=[8000,8000,8000,8000,65535,64556]


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


def accepted_pregrasp(path,expected_sha=None):
    raw=Path(path).read_bytes();digest=hashlib.sha256(raw).hexdigest()
    if expected_sha is not None and digest!=expected_sha:raise ValueError('Pregrasp changed after parent qualification')
    with zipfile.ZipFile(io.BytesIO(raw)) as z:
        names=z.namelist();pre=names[0].split('/')[0]+'/'
        get=lambda name:json.loads(z.read(pre+name))
        hashes=get('source_hashes.json');parent=get('result.json');worker=get('worker_result.json')
        if (z.testzip() or len(names)!=len(set(names))
                or set(hashes)!={n[len(pre):] for n in names if n!=pre+'source_hashes.json'}
                or any(hashlib.sha256(z.read(pre+n)).hexdigest()!=h for n,h in hashes.items())):
            raise ValueError('Pregrasp CRC/hash inventory differs')
        deployed={name:h for h,name in (line.split('  ',1) for line in
            (ROOT/'deployment/visual-pick-sort-v11-files.sha256').read_text(encoding='ascii').splitlines())}
        events=[json.loads(line) for line in z.read(pre+'operator_events.jsonl').splitlines()]
        if (parent.get('deployed_source_sha256')!=deployed or parent.get('worker')!=worker
                or parent.get('scope')!='fresh_live_target_left2p8mm_plus_lower_left4p5mm_pregrasp'
                or parent.get('outcome')!='visual_pregrasp_alignment_confirmed_by_operator'
                or parent.get('worker_returncode')!=0 or parent.get('exceptions')
                or parent.get('operator_endpoint_verdict')!='p'
                or parent.get('new_target_physical_alignment_accepted') is not True
                or [e.get('value') for e in events]!=['m','d','p']
                or worker.get('phase')!='completed' or worker.get('exceptions') or worker.get('cleanup_errors')
                or worker.get('pregrasp_endpoint_feedback_completed') is not True
                or worker.get('grasp_performed') is not False or worker.get('no_grasp_or_lift') is not True
                or len(worker.get('arm_segments',[])) not in (3,4)
                or worker.get('arm_actuator_writes')!=len(worker.get('arm_segments',[]))
                or worker.get('hand_register_writes')!=1 or worker.get('controller_configuration_writes')!=0):
            raise ValueError('Clean actual visual pregrasp and operator p required; no fault source')
        segments=worker['arm_segments']
        expected_phases=(['raising_current_empty_hand_before_alignment'] if worker.get('startup_raise_required') else [])+[
            'aligning_above_visual_target','approaching_visual_hover_140mm','descending_to_visual_noncontact_pregrasp']
        if [row['phase'] for row in segments]!=expected_phases:
            raise ValueError('Fresh visual route must match actual initial-height branch')
        arm=worker['final_arm'];last=segments[-1]
        if (last['phase']!='descending_to_visual_noncontact_pregrasp' or last['endpoint']!=arm
                or not all(s['notification']['acknowledged'] and s['notification']['event_seen'] for s in worker['arm_segments'])
                or arm['arm_error']!=0 or arm['system_error']!=0
                or any(abs(a-b)>2 for a,b in zip(worker['last_snapshot']['targets'],PREPARE))
                or parent['grasp_offset']!=worker['grasp_offset']):
            raise ValueError('Accepted endpoint/hand/correction provenance differs')
        return dict(archive_sha256=digest,arm=arm,hand=worker['last_snapshot'],
            grasp_offset=worker['grasp_offset'],configuration=worker['observed_run_configuration'],
            pregrasp_write_counts=dict(arm=len(segments),hand=1,configuration=0),
            actual_visual_pregrasp_accepted=True,old_fault_last_arm_state_used=False,
            grip_or_lift_or_bin_accepted=False)


def grip_recipe_proof():
    path=ROOT/'deployment/visual_binding_grasp_source_123051.json';raw=path.read_bytes()
    if hashlib.sha256(raw).hexdigest()!='6652427ab133c3d48ed334f548595b523874436f16d174d833a596b634848614':
        raise ValueError('Historical grip receipt changed')
    source=json.loads(raw);receipt=source['new_grip_receipt']
    if (receipt['recipe']!=RECIPE or len(receipt['groups'])!=3
            or receipt['old_fault_is_not_motion_authority'] is not True
            or any(any(s in (3,4) for s in row['status'])
                or any(abs(a-b)>2 for a,b in zip(row['targets'],RECIPE))
                or any(abs(a-b)>150 for a,b in zip(row['positions'],RECIPE)) for row in receipt['groups'])):
        raise ValueError('Healthy historical pre-lift grip recipe receipt required')
    return dict(source_sha256=hashlib.sha256(raw).hexdigest(),recipe=RECIPE[:],
        receipt=receipt,scope='recipe_only_no_old_arm_position_or_fault_state_authority')
