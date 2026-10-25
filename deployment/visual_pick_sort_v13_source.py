"""File provenance for the newly taught wrist; no drivers or actuator authority."""
import hashlib
import json
import math
from pathlib import Path
import zipfile

ROOT=Path(__file__).resolve().parents[1]
MANIFEST=ROOT/'deployment/visual-pick-sort-v13-files.sha256'
SOURCE_ANCHOR=ROOT/'deployment/new_grasp_reference_200928/new_task_anchor.json'
SOURCE_ANCHOR_SHA='b6405647fe47db40885818163115e9b54f1acb8030f2faac01bcb004eacae8a5'
PRE=[35000,7500,9480,30000,65535,65535]
GRIP=[40000,12500,14480,30000,65535,65535]
DROP=[0,0,0,0,0,65535]


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_delivery():
    inventory={}
    for line in MANIFEST.read_text(encoding='ascii').splitlines():
        digest,name=line.split('  ',1);path=(ROOT/name).resolve()
        if ROOT not in path.parents or name in inventory or sha(path)!=digest:
            raise ValueError('Deployment file changed: '+name)
        inventory[name]=digest
    if not inventory:raise ValueError('Empty deployment manifest')
    return inventory


def accepted_anchor():
    if (ROOT/'calibration/new_wrist_reference_pending.json').exists():
        raise ValueError('旧腕姿已停用。请先完成r/u/v新腕姿记录并接入新参考；不能继续运行旧视觉动作。')
    if sha(SOURCE_ANCHOR)!=SOURCE_ANCHOR_SHA:raise ValueError('New200928 anchor source differs')
    anchor=json.loads(SOURCE_ANCHOR.read_bytes())
    arm_file=SOURCE_ANCHOR.parent/'arm_reference_002.json'
    if sha(arm_file)!=anchor['provenance']['arm_reference_sha256']:
        raise ValueError('New arm reference source differs')
    arm=json.loads(arm_file.read_bytes())
    point=json.loads((SOURCE_ANCHOR.parent/'reference_point.json').read_bytes())
    if (anchor.get('schema_version')!=2 or anchor.get('task_binding_recorded') is not True
            or anchor.get('fault_last_state_used') is not False
            or anchor.get('old_grasp_tcp_mapping_used') is not False
            or anchor.get('old_2p8_plus_4p5_offsets_applied') is not False
            or anchor.get('reference_arm')!=arm.get('reference_arm')
            or arm.get('reference_qualified') is not True or arm.get('SDK_cleanup_finished') is not True
            or arm.get('exceptions') or arm.get('cleanup_errors')
            or anchor['reference_burst']!=point['burst_summary']
            or anchor['reference_top_center_world_m']!=point['top_center_world_m']
            or anchor['camera_context']!=point['binding_camera_context']
            or anchor['equal_height_plane']!=point['equal_height_plane']
            or anchor['recipe']['pregrasp_targets']!=PRE or anchor['recipe']['grasp_targets']!=GRIP):
        raise ValueError('Qualified current new-wrist reference chain required')
    return anchor


def read_point_archive(path):
    path=Path(path)
    with zipfile.ZipFile(path) as archive:
        if archive.testzip() is not None:raise ValueError('Current visual archive is incomplete')
        names=archive.namelist()
        manifests=[n for n in names if n.endswith('/source_hashes.json')]
        if len(manifests)!=1 or len(names)!=len(set(names)):
            raise ValueError('Unique visual archive and hash manifest required')
        prefix=manifests[0][:-len('source_hashes.json')]
        inventory=json.loads(archive.read(manifests[0]))
        for name,digest in inventory.items():
            if '/' in name or '\\' in name or hashlib.sha256(archive.read(prefix+name)).hexdigest()!=digest:
                raise ValueError('Visual evidence file differs: '+name)
        point=json.loads(archive.read(prefix+'latest_reference_point.json'))
        report=json.loads(archive.read(prefix+'result.json'))
    if report.get('error') or report.get('cleanup_errors'):
        raise ValueError('Current visual capture has an error or incomplete cleanup')
    for name,digest in report['runtime_source_sha256'].items():
        if sha(ROOT/'deployment'/name)!=digest:raise ValueError('Current vision source differs: '+name)
    return point,dict(archive_sha256=sha(path),source=str(path),hashed_files=len(inventory),
        capture_outcome=report['outcome'],SDK_loaded=report['robot_SDK_loaded'],
        arm_writes=report['arm_writes'],hand_writes=report['hand_writes'])


def accepted_target(record):
    from new_grasp_task_anchor_v1 import target_candidate
    anchor=accepted_anchor()
    if record.get('source_anchor_sha256')!=SOURCE_ANCHOR_SHA or record.get('anchor')!=anchor:
        raise ValueError('Current target is not bound to the accepted new wrist')
    expected=target_candidate(record['point_record'],anchor)
    if record.get('candidate')!=expected:raise ValueError('Current target candidate was altered')
    permissions=record.get('scene_permission',{})
    for key in ('hand_empty','inert_shell','fixed_head_board_mount','exclusive','onsite_stop',
                'whole_route_min20mm_clear','continuous_cycle_authorized'):
        if permissions.get(key) is not True:raise ValueError('Current scene permission missing: '+key)
    return anchor,expected


def back40_offset():
    transform=accepted_anchor()['equal_height_plane']['world_from_camera']
    x,y=transform[0][2],transform[1][2];length=math.hypot(x,y)
    if not math.isfinite(length) or length<1e-9:raise ValueError('Current fixed camera horizontal direction invalid')
    return [-.040*x/length,-.040*y/length,0.]
