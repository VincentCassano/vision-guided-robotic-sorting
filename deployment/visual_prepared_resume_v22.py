"""File provenance for continuing the known post-PRE, pre-GRIP interruption."""
import copy
import hashlib
import json
from pathlib import Path

PRIOR_MANIFEST_SHA='908bf9dba01b43f5bdf6b97db431dc5607ed8144a92c16e99f08d8e3ebe28d93'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_prepared_pick(root,path=None):
    root=Path(root).resolve();captures=root/'captures'
    if path is None:
        files=list(captures.glob('visual_pick_sort_v22_*/cycle_*/worker_result.json'))
        if not files:raise ValueError('No saved post-PRE cycle to continue')
        path=max(files,key=lambda p:p.stat().st_mtime)
    path=Path(path).resolve()
    if captures not in path.parents or path.name!='worker_result.json':
        raise ValueError('Prepared source must remain inside captures')
    request_path=path.with_name('request.json')
    original=json.loads(request_path.read_bytes());result=json.loads(path.read_bytes())
    sealed=root/'deployment/visual_pick_sort_v22_fix3_prior_manifest.sha256'
    if sha(sealed)!=PRIOR_MANIFEST_SHA:raise ValueError('Prior fix2 manifest differs')
    sources={name:digest for digest,name in
        (line.split('  ',1) for line in sealed.read_text(encoding='ascii').splitlines())}
    errors=result.get('exceptions',[]);motions=result.get('motion_requests',[])
    commands=result.get('hand_commands',[])
    prepared=result.get('prepare_after_descent_targets')
    endpoint=result.get('qualified_descent_endpoint')
    if (original.get('mode')!='visual_cycle' or result.get('mode')!='visual_cycle'
            or result.get('phase')!='fault'
            or result.get('failed_phase')!='preparing_after_descent_at_visual_target'
            or len(errors)!=1 or not errors[0].get('error','').startswith(
                'RuntimeError("arm unchanged during postdescent PRE6:')
            or result.get('request_sha256')!=sha(request_path)
            or original.get('manifest_sha256')!=PRIOR_MANIFEST_SHA
            or result.get('delivery_manifest_sha256')!=PRIOR_MANIFEST_SHA
            or result.get('delivery_source_sha256')!=sources
            or result.get('SDK_cleanup_finished') is not True or result.get('cleanup_errors')
            or result.get('arm_stop_writes')!=0 or result.get('controller_configuration_writes')!=0
            or result.get('qualified_loaded_upper') or result.get('qualified_bin_endpoint')
            or result.get('grip_together_targets') or not isinstance(endpoint,dict)
            or not motions or motions[-1].get('phase')!='descending_live_visual_target'
            or motions[-1].get('qualified_endpoint')!=endpoint
            or result.get('arm_actuator_writes')!=len(motions)
            or len(commands)!=1 or commands[0].get('label')!='prepare_after_descent'
            or commands[0].get('acknowledged') is not True
            or result.get('hand_register_writes')!=1
            or prepared!=original.get('candidate',{}).get('prepare_recipe')
            or result.get('current_hand_targets')!=prepared
            or result.get('accepted_candidate')!=original.get('candidate')):
        raise ValueError('Latest action is not the sealed post-PRE interruption; no original-pose continuation')
    if (endpoint.get('arm_error')!=0 or endpoint.get('system_error')!=0
            or min(endpoint.get('reserve',[-1]))<3
            or min(endpoint.get('drive_reserve',[-1]))<3):
        raise ValueError('Original completed descent endpoint was not qualified')
    proof=dict(original_request_file=str(request_path),original_result_file=str(path),
        original_request_sha256=sha(request_path),original_result_sha256=sha(path),
        original_manifest_sha256=PRIOR_MANIFEST_SHA,
        completed_descent_endpoint=copy.deepcopy(endpoint),
        expected_prepare_targets=prepared[:],fault_last_arm_state_used=False,
        resend_PRE=False,new_descent=False)
    return copy.deepcopy(original),proof
