#!/usr/bin/env python3
"""Audit one immutable field capture and record an operator correction offline.

No SDK import, robot connection, installation change or motion authorization.
The rear camera measurement does not certify adapter/connector coverage.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/battery_grasp/src'))
from battery_grasp.pregrasp_measurements import parse_length, summarize_measurements

CAPTURE_SHA = '77f1233b9dfcd45a198c37b599599b81b9daf873dfd9dde2084f0510fff36012'
PACKAGE_SHA = '89f039e6fc734c0e5f72c372f43d6eaa55b0c4e7c7deed71384f36990a2c9752'
GETS = {'Get_Current_Work_Frame', 'Get_Current_Tool_Frame',
        'Get_Current_Arm_State', 'Get_Joint_Speed', 'Get_Joint_Acc',
        'Get_Joint_Min_Pos', 'Get_Joint_Max_Pos'}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def review(capture, rear_distance_mm):
    rear_distance_mm = parse_length(rear_distance_mm)
    require(rear_distance_mm is not None, 'A measured rear-camera distance is required')
    require(rear_distance_mm == 85, 'This provenance record covers only the operator-reported85mm correction')
    raw = capture.read_bytes()
    require(digest(raw) == CAPTURE_SHA, 'This review is bound to the 162445 original capture')
    package = ROOT / 'deployment/battery-pregrasp-measurements-v1-20261012.zip'
    require(digest(package.read_bytes()) == PACKAGE_SHA, 'Measurement package changed')
    with zipfile.ZipFile(package) as archive:
        manifest = archive.read('deployment/pregrasp-measurement-files.sha256')
    expected = {name: sha for sha, name in
                (line.split('  ', 1) for line in manifest.decode('ascii').splitlines())}
    require(len(expected) == 163, 'Expected the reviewed 163-file installation')
    # Audit against the frozen package chain already present locally.
    for name, sha in expected.items():
        path = ROOT / name if name != 'start_robot_d435.sh' else ROOT / 'deployment' / name
        require(digest(path.read_bytes()) == sha, 'Local frozen payload changed: ' + name)

    with zipfile.ZipFile(capture) as archive:
        require(archive.testzip() is None, 'Capture CRC failed')
        names = archive.namelist()
        require(len(names) == 20 and len(set(names)) == 20, 'Unexpected capture members')
        require(all('..' not in Path(n).parts for n in names), 'Unsafe member')
        roots = [n for n in names if n.count('/') == 1 and n.endswith('/result.json')]
        require(len(roots) == 1, 'A single parent result is required')
        prefix = roots[0].rsplit('/', 1)[0]
        read_json = lambda name: json.loads(archive.read(prefix + '/' + name))
        read_rows = lambda name: [json.loads(line) for line in
                                 archive.read(prefix + '/' + name).decode('utf-8').splitlines()]
        original = read_json('result.json')
        require(original['worksheet'] == read_json('measurements.json'), 'Worksheet copies differ')
        require(original['installed_file_sha256'] == expected, 'Field install manifest differs')
        require(original['outcome'] == 'physical_measurements_collected_descent_pending'
                and not original['exceptions'] and original['hardware_writes'] == 0
                and original['execution_ready'] is False, 'Parent scope/outcome failed')
        worker_audits = {}
        for side in ('before', 'after'):
            worker = read_json(side + '/result.json')
            embedded = copy.deepcopy(original[side])
            process = embedded.pop('process')
            require(embedded == worker, 'Embedded worker differs: ' + side)
            require(process == read_json(side + '/process_status.json')
                    and process['return_code'] == 0, 'Worker process failed: ' + side)
            require(worker['exceptions'] == [] and worker['static_readout_valid'] is True
                    and worker['geometry_reference_eligible'] is True
                    and worker['hardware_writes'] == worker['arm_actuator_writes']
                    == worker['hand_register_writes'] == 0
                    and worker['execution_ready'] is False, 'Worker scope failed: ' + side)
            sdk = read_rows(side + '/sdk_calls.jsonl')
            calls = [row for row in sdk if row['event'] == 'sdk_call_before']
            returns = [row for row in sdk if row['event'] == 'sdk_call_after']
            require(len(calls) == len(returns) == 16, 'SDK events incomplete')
            require([r['method'] for r in calls] == [r['method'] for r in returns], 'SDK ordering differs')
            require({r['method'] for r in calls} == GETS | {'Arm_Socket_Close', 'RM_API_UnInit'},
                    'Non-read SDK method in capture')
            for method in GETS:
                results = [r['value'] for r in returns if r['method'] == method]
                require(len(results) == 2 and all(r[0] == 0 for r in results), 'Failed GET: ' + method)
                for phase, value in zip(('arm_before', 'arm_after'), results):
                    require(worker[phase][method]['data'] == value[1:], 'SDK result does not match worker')
            wire = read_rows(side + '/wire.jsonl')
            requests = [r for r in wire if r['event'] == 'tx']
            replies = [r for r in wire if r['event'] == 'rx']
            decoded = [r for r in wire if r['event'] == 'response']
            require(len(requests) == len(replies) == len(decoded) == 44, 'Hand transport incomplete')
            for sequence, (tx, rx, response) in enumerate(zip(requests, replies, decoded), 1):
                require(tx['sequence'] == rx['sequence'] == response['sequence'] == sequence,
                        'Hand sequence differs')
                request = json.loads(bytes.fromhex(tx['hex']))
                reply = json.loads(bytes.fromhex(rx['hex']))
                require(request == tx['request'] and reply == response['response'], 'Raw hand JSON differs')
                require(request['command'] in {'read_multiple_holding_registers', 'read_holding_registers'},
                        'A non-read hand request was recorded')
                values = reply['data']
                if request['command'] == 'read_multiple_holding_registers':
                    require(len(values) == 2 * request['num'], 'Unexpected hand word count')
                    words = [values[i] * 256 + values[i+1] for i in range(0, len(values), 2)]
                    require(words == response['value'], 'Hand word decode differs')
                else:
                    require(values == response['value'], 'Single-register decode differs')
            worker_audits[side] = dict(process_return_code=0, query_calls=14, cleanup_calls=2,
                hand_read_requests=44, raw_request_response_matches=True,
                static_readout_valid=True, geometry_reference_eligible=True,
                hardware_writes=0, arm_native_wire_captured=False,
                hand_pose=worker['analysis']['hand_pose_classification'])

    before, after = original['before'], original['after']
    replay = summarize_measurements(original['worksheet'], before, after)
    require(replay == original['analysis'], 'Original analysis is not reproducible')
    corrected = copy.deepcopy(original['worksheet'])
    key = 'camera_adapter_max_distance_from_rim_P_mm'
    old = corrected['lengths'][key]
    corrected['lengths'][key] = rear_distance_mm
    analysis = summarize_measurements(corrected, before, after)
    a = before['analysis']['current_arm']
    b = after['analysis']['current_arm']
    audit = dict(schema_version=1, source_capture=capture.name, source_sha256=digest(raw),
        source_bytes=len(raw), source_member_count=20, source_crc_pass=True,
        source_archive_preserved=True, installation_file_count=163, installation_matches=True,
        original_analysis_reproduced=True, worker_audits=worker_audits,
        correction=dict(field=key, original_mm=old, corrected_mm=rear_distance_mm,
            source='direct_user_message_2026-10-13',
            scope='same_P_to_rear_camera_shell_outermost_edge',
            adapter_connector_full_coverage_confirmed=False,
            correction_pose_independently_bracketed=False,
            raw_capture_not_changed=True),
        measurement_pose_tcp_drift_mm=math.dist(a['tcp_position_mm'], b['tcp_position_mm']),
        measurement_pose_max_joint_drift_degrees=max(abs(x-y) for x,y in
            zip(a['joint_degrees'], b['joint_degrees'])),
        minimum_current_joint_reserve_degrees=min(b['joint_reserve_degrees']),
        worksheet=corrected, analysis=analysis,
        audit_pass=True, execution_ready=False, hardware_writes=0,
        evidence_limits=[
            'Two endpoint snapshots and the operator report support measurement pose consistency; no continuous motion monitor.',
            'Recorded OPEN width19.4mm versus battery45.6mm is one cross-section; not proof of PREPARE incompatibility.',
            'No cable segments supplied, despite completeness answer true; do not infer that external cables are absent.',
            'P description contains encoding replacement characters; chat description remains a separate source.',
            '85mm is an operator correction after the capture; do not certify maximum of all adapter/connector surfaces.',
            'Scene scalar lengths do not supply World obstacle placements, detailed PREPARE aperture or controller path.',
        ],
        next_development='Reuse accepted numeric routes and measured dimensions; refine actual PREPARE and installed/scene geometry, then controller execution entry. No repeat collection requested by this review.')
    require(digest(capture.read_bytes()) == digest(raw), 'Source was altered')
    require(analysis['gross_hand_sphere_candidate']['radius_mm'] ==
            replay['gross_hand_sphere_candidate']['radius_mm'], 'Unexpected sphere change for this correction')
    return corrected, audit


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--camera-rear-distance-mm', required=True, type=float)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args(argv)
    worksheet, audit = review(args.capture, args.camera_rear_distance_mm)
    args.output.mkdir(parents=True, exist_ok=True)
    for filename, data in (('corrected_measurements.json', worksheet), ('review.json', audit)):
        path = args.output / filename
        content = (json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n').encode('utf-8')
        require(not path.exists() or path.read_bytes() == content, 'Refusing to replace different review: ' + filename)
        path.write_bytes(content)
    print('AUDIT PASS; camera rear={}mm; current hand OPEN; no hardware connection; EXECUTION_READY=False'.format(args.camera_rear_distance_mm))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
