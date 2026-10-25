"""File-only proof of the fix7 failure or its fix8 continuation failure.

Only a successful pre-transfer endpoint may become a comparison reference.
The fault last_arm_state and successful ACKs never qualify an endpoint.
"""
import copy
import hashlib
import json
import math
from pathlib import Path

PRIOR_MANIFEST_SHA='c73389bcfb4dc9460f8319a700ed67a662b30891554eea6698c2a3b9a8dee486'
FIX8_MANIFEST_SHA='6d2e08811b3ee24fa92d8713576c7a9bc9f587623723efbe2c870b843c4ffe38'
FIX9_MANIFEST_SHA='7c67db60b5731aa05ba54bf1908ba42f298e8a6328d0ba208c2f23f38f4a9fc0'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def protection_only_fix9_startup(root,path):
    """Only the sealed, zero-effect park startup failure may be skipped by c."""
    request_path=path.with_name('request.json')
    result=json.loads(path.read_bytes());request=json.loads(request_path.read_bytes())
    if result.get('delivery_version')!='visual-pick-sort-v23-fix9-20261025':
        return False
    sealed=root/'deployment/visual_pick_sort_v23_fix10_fix9_manifest.sha256'
    if sha(sealed)!=FIX9_MANIFEST_SHA:raise ValueError('Sealed fix9 source manifest differs')
    sources={name:digest for digest,name in
        (line.split('  ',1) for line in sealed.read_text(encoding='ascii').splitlines())}
    expected={'Get_Collision_Stage':[0,8],'Get_Self_Collision_Enable':[0,False]}
    errors=result.get('exceptions',[])
    return (request.get('mode')=='park_empty' and result.get('mode')=='park_empty'
        and result.get('phase')=='fault' and result.get('failed_phase')=='preflight'
        and result.get('request_sha256')==sha(request_path)
        and request.get('manifest_sha256')==FIX9_MANIFEST_SHA
        and result.get('delivery_manifest_sha256')==FIX9_MANIFEST_SHA
        and result.get('delivery_source_sha256')==sources
        and result.get('protection_before')==expected and result.get('protection_after')==expected
        and all(type(result.get(k)) is int and result[k]==0 for k in
            ('arm_actuator_writes','hand_register_writes','arm_stop_writes','controller_configuration_writes'))
        and result.get('sdk_calls')==[] and result.get('motion_requests')==[]
        and result.get('hand_commands')==[] and result.get('SDK_cleanup_finished') is False
        and not path.with_name('worker_result_hand.jsonl').exists()
        and len(errors)==1 and errors[0].get('error','').startswith(
            'RuntimeError("Current protection must remain collision8 / selfTrue:')
        and all(e.get('method')=='final_readonly_configuration' and e.get('error','').startswith(
            'RuntimeError("Current protection must remain collision8 / selfTrue:')
            for e in result.get('cleanup_errors',[])))


def load_failed_transfer(root,path=None):
    root=Path(root).resolve();captures=root/'captures'
    if path is None:
        files=sorted(captures.glob('visual_pick_sort_v23_*/cycle_*/worker_result.json'),
            key=lambda p:p.stat().st_mtime,reverse=True)
        if not files:raise ValueError('No saved transfer result')
        for candidate in files:
            if protection_only_fix9_startup(root,candidate):
                print('续投略过仅保护检查且臂手0动作的启动记录：'+str(candidate),flush=True)
                continue
            path=candidate;break
        if path is None:raise ValueError('Only zero-action protection startup records; no held transfer proof')
    path=Path(path).resolve();request_path=path.with_name('request.json')
    hand_path=path.with_name('worker_result_hand.jsonl')
    if captures not in path.parents or path.name!='worker_result.json':
        raise ValueError('Transfer source must be an original result inside captures')
    original=json.loads(request_path.read_bytes());result=json.loads(path.read_bytes())
    if result.get('delivery_version')=='visual-pick-sort-v23-fix8-20261024':
        return load_fix8_failure(root,path,request_path,hand_path,original,result)
    sealed=root/'deployment/visual_pick_sort_v23_fix8_prior_manifest.sha256'
    if sha(sealed)!=PRIOR_MANIFEST_SHA:raise ValueError('Sealed fix7 source manifest differs')
    sources={name:digest for digest,name in
        (line.split('  ',1) for line in sealed.read_text(encoding='ascii').splitlines())}
    connected=result.get('connected_transfer',{});motions=result.get('motion_requests',[])
    if (original.get('mode')!='visual_cycle' or original.get('wrist_profile')!='near'
            or result.get('delivery_version')!='visual-pick-sort-v23-fix7-20261024'
            or result.get('mode')!='visual_cycle' or result.get('phase')!='fault'
            or result.get('failed_phase')!='connected_school_selected_bin_transfer'
            or result.get('request_sha256')!=sha(request_path)
            or original.get('manifest_sha256')!=PRIOR_MANIFEST_SHA
            or result.get('delivery_manifest_sha256')!=PRIOR_MANIFEST_SHA
            or result.get('delivery_source_sha256')!=sources
            or result.get('SDK_cleanup_finished') is not True or result.get('cleanup_errors')
            or result.get('controller_configuration_writes')!=0
            or result.get('arm_stop_writes')!=1 or result.get('fault_stop_raw')!=0
            or connected.get('events')!=[] or connected.get('acknowledgements')!=[0,0,0]
            or connected.get('flags')!=[1,1,0] or result.get('qualified_bin_endpoint')
            or result.get('post_drop_open_posture') or len(motions)<4):
        raise ValueError('Source is not the known fix7 pre-transfer failure; no held continuation')
    upper=result.get('qualified_common_loaded_upper');last=motions[-4:]
    if (not isinstance(upper,dict) or last[0].get('phase')!='carrying_to_verified_common_upper'
            or last[0].get('method')!='Movej_Cmd' or last[0].get('qualified_endpoint')!=upper
            or [m.get('method') for m in last[1:]]!=['Movej_P_Cmd']*3
            or [m.get('trajectory_connect') for m in last[1:]]!=[1,1,0]
            or any(m.get('qualified_endpoint') is not None for m in last[1:])
            or upper.get('arm_error')!=0 or upper.get('system_error')!=0
            or min(upper.get('reserve',[-1]))<3 or min(upper.get('drive_reserve',[-1]))<3):
        raise ValueError('No successful pre-transfer common-upper endpoint')
    commands=result.get('hand_commands',[]);attempted=[c for c in commands if c.get('attempted')]
    if (not attempted or any(c.get('acknowledged') is not True for c in attempted)
            or any(c.get('label')=='normal_bin_open' for c in commands)
            or result.get('hand_register_writes')!=len(attempted)):
        raise ValueError('Original grip command proof or no-release condition differs')
    expected=original.get('candidate',{}).get('grip_recipe')
    current=result.get('last_hand_working',{}).get('targets')
    if (not isinstance(expected,list) or len(expected)!=6 or not isinstance(current,list)
            or len(current)!=6 or any(abs(a-b)>2 for a,b in zip(current,expected))):
        raise ValueError('Original held targets unavailable')
    calls=[c for c in result.get('sdk_calls',[]) if c.get('method')=='Movej_P_Cmd']
    if len(calls)!=3 or any(c.get('result')!=0 or c.get('args',[None])[0]!=m.get('destination')
            for c,m in zip(calls,last[1:])):
        raise ValueError('Saved transfer commands differ from their ACK records')
    rows=[json.loads(line) for line in hand_path.read_text(encoding='utf8').splitlines() if line.strip()]
    events=[r for r in rows if r.get('event')=='external_arm_telemetry_ignored_by_manual_hand_tool'
        and r.get('monotonic',-1)>=calls[0]['monotonic']]
    if (len(events)!=1 or events[0].get('notification')!={'device':0,'state':'current_trajectory_state',
            'trajectory_connect':0,'trajectory_state':False}):
        raise ValueError('Expected one real controller failure, with no completed transfer segment')
    for value in (upper.get('pose'),upper.get('joints')):
        if not isinstance(value,list) or len(value)!=6 or any(type(v) not in (int,float) or not math.isfinite(v) for v in value):
            raise ValueError('Malformed completed common-upper reference')
    proof=dict(original_result_file=str(path),original_request_file=str(request_path),
        original_result_sha256=sha(path),original_request_sha256=sha(request_path),
        original_hand_log_sha256=sha(hand_path),original_manifest_sha256=PRIOR_MANIFEST_SHA,
        qualified_common_upper=copy.deepcopy(upper),expected_grip_targets=expected[:],
        controller_failure_notification=copy.deepcopy(events[0]),
        failed_last_arm_state_used=False,no_new_grip_or_lift=True,
        release_not_previously_commanded=True)
    return copy.deepcopy(original),proof


def load_fix8_failure(root,path,request_path,hand_path,request,result):
    sealed=root/'deployment/visual_pick_sort_v23_fix9_fix8_manifest.sha256'
    if sha(sealed)!=FIX8_MANIFEST_SHA:raise ValueError('Sealed fix8 source manifest differs')
    sources={name:digest for digest,name in
        (line.split('  ',1) for line in sealed.read_text(encoding='ascii').splitlines())}
    prior=request.get('transfer_failure_provenance',{})
    parent_file=prior.get('original_result_file')
    if not isinstance(parent_file,str) or not parent_file or Path(parent_file).resolve()==path:
        raise ValueError('Invalid fix7 ancestor of fix8 failure')
    original,root_proof=load_failed_transfer(root,parent_file)
    if prior!=root_proof:raise ValueError('Original completed common-upper proof differs')
    connected=result.get('connected_transfer',{});motions=result.get('motion_requests',[])
    if (request.get('mode')!='resume_failed_transfer' or result.get('mode')!='resume_failed_transfer'
            or result.get('phase')!='fault' or result.get('failed_phase')!='connected_school_selected_bin_transfer'
            or result.get('request_sha256')!=sha(request_path)
            or request.get('manifest_sha256')!=FIX8_MANIFEST_SHA
            or result.get('delivery_manifest_sha256')!=FIX8_MANIFEST_SHA
            or result.get('delivery_source_sha256')!=sources
            or result.get('SDK_cleanup_finished') is not True or result.get('cleanup_errors')
            or result.get('controller_configuration_writes')!=0 or result.get('hand_register_writes')!=0
            or result.get('hand_commands') or result.get('arm_stop_writes')!=1 or result.get('fault_stop_raw')!=0
            or connected.get('events')!=[] or connected.get('acknowledgements')!=[0,0,0]
            or connected.get('flags')!=[1,1,0] or result.get('qualified_bin_endpoint')
            or result.get('post_drop_open_posture') or len(motions)!=3
            or [m.get('method') for m in motions]!=['Movej_P_Cmd']*3
            or [m.get('trajectory_connect') for m in motions]!=[1,1,0]
            or result.get('no_new_grip_or_lift_during_resume') is not True):
        raise ValueError('Latest fix8 record is not the unreleased pre-transfer failure')
    for key in ('wrist_profile','success_source_sha256','anchor','near_binding','wrist_regions',
            'point_record','candidate','target_classification'):
        if request.get(key)!=original.get(key):raise ValueError('Original held target differs: '+key)
    calls=[c for c in result.get('sdk_calls',[]) if c.get('method')=='Movej_P_Cmd']
    if len(calls)!=3 or any(c.get('result')!=0 or c.get('args',[None])[0]!=m.get('destination')
            for c,m in zip(calls,motions)):
        raise ValueError('Saved fix8 transfer calls differ')
    rows=[json.loads(line) for line in hand_path.read_text(encoding='utf8').splitlines() if line.strip()]
    events=[r for r in rows if r.get('event')=='external_arm_telemetry_ignored_by_manual_hand_tool'
        and r.get('monotonic',-1)>=calls[0]['monotonic']]
    failed=[e for e in events if e.get('notification')=={'device':0,'state':'current_trajectory_state',
        'trajectory_connect':0,'trajectory_state':False}]
    if not failed or any(e.get('notification',{}).get('trajectory_connect')==1 for e in events):
        raise ValueError('Actual fix8 failure before any completed transfer segment is unproven')
    proof=copy.deepcopy(root_proof)
    proof.update(original_result_file=str(path),original_request_file=str(request_path),
        original_result_sha256=sha(path),original_request_sha256=sha(request_path),
        original_hand_log_sha256=sha(hand_path),original_manifest_sha256=FIX8_MANIFEST_SHA,
        controller_failure_notification=copy.deepcopy(failed[-1]),
        original_fix7_completed_motion_proof=copy.deepcopy(root_proof),
        latest_failure_is_not_a_new_endpoint=True)
    return copy.deepcopy(original),proof
