"""Accept only the completed near lift before the known v21 variable error.

File validation only. The fault last_arm_state is never a motion reference.
"""
import copy
import hashlib
import json
import math
from pathlib import Path

ORIGINAL_MANIFEST_SHA='a6954a6aa26afcac75d7573d3eb1c3711ee1786c35991d83b04fb780681b4661'
ORIGINAL_WORKER_SHA='709665d94c0fd2abf70be631f4bb24e9dc1b7ce769b3a2ec2fd2618fab5737a1'
FIX1_MANIFEST_SHA='6635e2580e3bfb5755a03a19fab2e755afed85601535be151b9c16f9da6ca2a9'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_completed_lift(root,path=None):
    from visual_pick_sort_v20_source import same_measurement
    root=Path(root).resolve();captures=root/'captures'
    if path is None:
        files=list(captures.glob('visual_pick_sort_v21_*/cycle_*/worker_result.json'))
        if not files:raise ValueError('No saved v21 cycle result; no movement')
        path=max(files,key=lambda p:p.stat().st_mtime)
    path=Path(path).resolve()
    if captures not in path.parents or path.name!='worker_result.json':
        raise ValueError('Original result must be inside captures')
    request_path=path.with_name('request.json')
    original=json.loads(request_path.read_bytes());result=json.loads(path.read_bytes())
    sealed=root/'deployment/visual_pick_sort_v21_fix1_original_manifest.sha256'
    if sha(sealed)!=ORIGINAL_MANIFEST_SHA:raise ValueError('Original v21 source manifest differs')
    source={name:digest for digest,name in
        (line.split('  ',1) for line in sealed.read_text(encoding='ascii').splitlines())}
    errors=result.get('exceptions',[])
    if (original.get('mode')!='visual_cycle' or original.get('wrist_profile')!='near'
            or result.get('mode')!='visual_cycle' or result.get('wrist_profile')!='near'
            or result.get('phase')!='fault' or result.get('failed_phase')!='lifting_live_grip180mm'
            or result.get('request_sha256')!=sha(request_path)
            or result.get('delivery_manifest_sha256')!=ORIGINAL_MANIFEST_SHA
            or result.get('delivery_source_sha256')!=source
            or source.get('deployment/visual_pick_sort_v21_worker.py')!=ORIGINAL_WORKER_SHA
            or len(errors)!=1 or not errors[0].get('error','').startswith('UnboundLocalError(')
            or 'request' not in errors[0].get('error','')
            or result.get('SDK_cleanup_finished') is not True or result.get('cleanup_errors')
            or result.get('arm_stop_writes')!=0 or result.get('controller_configuration_writes')!=0
            or result.get('connected_transfer') or result.get('qualified_bin_endpoint')
            or not same_measurement(result.get('accepted_candidate'),original.get('candidate'))):
        raise ValueError('Result is not the known v21 post-lift variable error; no recovery movement')
    motions=result.get('motion_requests',[])
    upper=result.get('qualified_loaded_upper')
    if (not motions or not isinstance(upper,dict) or result.get('arm_actuator_writes')!=len(motions)
            or motions[-1].get('phase')!='lifting_live_grip180mm'
            or motions[-1].get('method')!='Movel_Cmd'
            or motions[-1].get('qualified_endpoint')!=upper):
        raise ValueError('No original qualified completed lift endpoint')
    last=motions[-1];start=last['start']['pose'];end=upper['pose'];destination=last['destination']
    if (not all(isinstance(v,list) and len(v)==6 and all(type(x) in (int,float) and math.isfinite(x) for x in v)
            for v in (start,end,destination))
            or abs(destination[2]-start[2]-.180)>1e-6
            or math.dist(end[:3],destination[:3])>.0005 or end[2]-start[2]<.1795
            or math.dist(end[:2],start[:2])>.001
            or upper.get('arm_error')!=0 or upper.get('system_error')!=0
            or min(upper.get('reserve',[-1]))<3 or min(upper.get('drive_reserve',[-1]))<3):
        raise ValueError('Saved lift endpoint is not the original completed180mm motion')
    commands=result.get('hand_commands',[])
    attempted=[c for c in commands if c.get('attempted') is True]
    if (not attempted or any(c.get('acknowledged') is not True for c in attempted)
            or result.get('hand_register_writes')!=len(attempted)
            or any(c.get('label')=='normal_bin_open' for c in commands)):
        raise ValueError('Uncertain hand targets or release already occurred; no held continuation')
    expected=original['candidate']['grip_recipe']
    current=result.get('last_hand_working',{}).get('targets')
    if (not isinstance(current,list) or len(current)!=6 or len(expected)!=6
            or any(abs(a-b)>2 for a,b in zip(current,expected))):
        raise ValueError('Original complete held grip feedback unavailable')
    proof=dict(original_request_file=str(request_path),original_result_file=str(path),
        original_request_sha256=sha(request_path),original_result_sha256=sha(path),
        original_manifest_sha256=ORIGINAL_MANIFEST_SHA,completed_lift_motion_index=len(motions)-1,
        completed_lift_endpoint=copy.deepcopy(upper),expected_current_grip_targets=expected[:],
        fault_last_arm_state_used=False,no_new_lift_or_grip=True)
    return copy.deepcopy(original),proof


def load_current_held_takeover(root,path=None):
    """Identify the zero-write resume rejection; its state is diagnostic only.

    A separate explicit t action must qualify its own new healthy live start.
    This does not convert the rejected pose into a completed-lift reference.
    """
    root=Path(root).resolve();captures=root/'captures'
    if path is None:
        files=list(captures.glob('visual_pick_sort_v21_*/cycle_*/worker_result.json'))
        if not files:raise ValueError('No saved v21 continuation result')
        path=max(files,key=lambda p:p.stat().st_mtime)
    path=Path(path).resolve()
    if captures not in path.parents or path.name!='worker_result.json':
        raise ValueError('Rejected continuation must be inside captures')
    request_path=path.with_name('request.json')
    request=json.loads(request_path.read_bytes());result=json.loads(path.read_bytes())
    sealed=root/'deployment/visual_pick_sort_v21_fix2_prior_manifest.sha256'
    if sha(sealed)!=FIX1_MANIFEST_SHA:raise ValueError('Original fix1 manifest differs')
    sources={name:digest for digest,name in
        (line.split('  ',1) for line in sealed.read_text(encoding='ascii').splitlines())}
    errors=result.get('exceptions',[])
    if (request.get('mode')!='resume_transfer' or result.get('mode')!='resume_transfer'
            or result.get('phase')!='fault' or result.get('failed_phase')!='preflight'
            or result.get('request_sha256')!=sha(request_path)
            or request.get('manifest_sha256')!=FIX1_MANIFEST_SHA
            or result.get('delivery_manifest_sha256')!=FIX1_MANIFEST_SHA
            or result.get('delivery_source_sha256')!=sources
            or result.get('SDK_cleanup_finished') is not True or result.get('cleanup_errors')
            or any(result.get(k)!=0 for k in ('arm_actuator_writes','hand_register_writes',
                'arm_stop_writes','controller_configuration_writes'))
            or result.get('motion_requests') or result.get('hand_commands')
            or len(errors)!=1 or not errors[0].get('error','').startswith('RuntimeError(')
            or 'actual qualified completed180mm held upper: current state differs' not in errors[0].get('error','')
            or result.get('fresh_startup',{}).get('source')!='fresh_healthy_current_bracket_not_fault_last_state'):
        raise ValueError('Latest result is not the zero-write fix1 reference mismatch; no takeover selected')
    original,proof=load_completed_lift(root,request.get('transfer_resume_provenance',{}).get('original_result_file'))
    if request.get('transfer_resume_provenance')!=proof:
        raise ValueError('Original completed-lift provenance differs')
    for key in ('wrist_profile','success_source_sha256','anchor','near_binding','wrist_regions','point_record','candidate'):
        if request.get(key)!=original.get(key):raise ValueError('Original held target changed: '+key)
    context=dict(rejected_request_file=str(request_path),rejected_result_file=str(path),
        rejected_request_sha256=sha(request_path),rejected_result_sha256=sha(path),
        rejected_manifest_sha256=FIX1_MANIFEST_SHA,prior_actuator_writes=0,
        rejected_state_used_as_start=False,original_endpoint_used_as_start=False,
        new_fresh_healthy_start_required=True,explicit_current_held_takeover_required=True)
    return original,proof,context
