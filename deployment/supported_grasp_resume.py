"""Stdlib-only provenance for the reviewed pre-grasp input-timeout capture.

No fault last_arm_state is returned as a reference. No hardware access, SDK,
kinematics, automatic pose alignment or extraction of archive paths occurs.
"""
import copy
import hashlib
import json
from pathlib import Path
import zipfile

SOURCE_SHA256='da8e0d1ef696273acf38a4d8bd4421cbb4bacfc40ec745f518061acc48d20be7'
PREFIX='supported_low_lift_20261018_015735_502572/'


def load_resume_source(path):
    path=Path(path)
    raw=path.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=SOURCE_SHA256:
        raise ValueError('Resume source differs from reviewed015735 capture; no connection')
    with zipfile.ZipFile(path) as archive:
        names=archive.namelist()
        if len(names)!=len(set(names)):
            raise ValueError('Duplicate capture member')
        worker=json.loads(archive.read(PREFIX+'worker_result.json'))
        parent=json.loads(archive.read(PREFIX+'parent_result.json'))
    if (worker.get('outcome')!='aborted_no_further_writes'
            or worker.get('permission_index')!=5 or worker.get('hand_register_writes')!=0
            or worker.get('arm_actuator_writes')!=2 or worker.get('cleanup_errors')
            or worker.get('controller_configuration_writes')!=0
            or worker.get('hand_bus_configuration_writes')!=0
            or worker.get('loaded_lift_performed') or worker.get('contact_freeze')
            or parent.get('worker_return_code')!=1
            or parent.get('exceptions')!=[dict(error="TimeoutError('Operator input expired; no automatic hand opening or return')")]
            or [row.get('index') for row in worker.get('operator_inputs',[])]!=[1,2,4]):
        raise ValueError('Source is not the reviewed zero-hand-write pre-grasp timeout')
    segments=worker.get('arm_segments',[])
    if [row.get('phase') for row in segments]!=['lifting_empty_100mm','descending_to_supported_shell']:
        raise ValueError('Two completed source arm segments required')
    endpoint=segments[1]['endpoint']
    if endpoint.get('arm_error')!=0 or endpoint.get('system_error')!=0:
        raise ValueError('Successful descent endpoint required')
    return dict(source_sha256=SOURCE_SHA256,source_file=str(path.resolve()),
        reference_source='completed descending_to_supported_shell endpoint, never fault last_arm_state',
        completed_descent=copy.deepcopy(endpoint),completed_upper=copy.deepcopy(segments[0]['endpoint']),
        sdk_environment=copy.deepcopy(worker['sdk_environment']),
        fresh_model=copy.deepcopy(worker['fresh_model']),
        configuration=copy.deepcopy(worker['configuration']),
        arm_safety_reads=copy.deepcopy(worker['arm_safety_reads']),
        live_tip_parameters=copy.deepcopy(worker['live_tip_parameters']),
        observed_prepare=copy.deepcopy(worker['last_preflight']),
        original_hand_write_count=0,original_next_action='await_grasp_g',
        original_outcome=worker['outcome'],old_clearance_is_not_current_measurement=True)
