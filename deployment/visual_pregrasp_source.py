"""Verify recorded live v4 target provenance before any device connection."""
import hashlib
import io
import json
import math
from pathlib import Path
import zipfile

ROOT=Path(__file__).resolve().parents[1]
PREPARE=[8000,8000,8000,8000,65535,65056]


def digest(raw):return hashlib.sha256(raw).hexdigest()


def pairs(items):
    result={}
    for key,value in items:
        if key in result:raise ValueError('Duplicate JSON key')
        result[key]=value
    return result


def parse(raw):
    return json.loads(raw,object_pairs_hook=pairs,
        parse_constant=lambda value:(_ for _ in ()).throw(ValueError('Nonfinite JSON')))


def vector(value,size):
    if (not isinstance(value,list) or len(value)!=size
            or any(type(v) not in (int,float) or not math.isfinite(v) for v in value)):
        raise ValueError('Malformed finite vector')
    return value[:]


def load_target(path):
    raw=Path(path).read_bytes()
    with zipfile.ZipFile(io.BytesIO(raw)) as archive:
        names=archive.namelist()
        if len(names)!=len(set(names)) or not names:raise ValueError('Duplicate or empty archive')
        prefix=names[0].split('/')[0]+'/'
        if any(not n.startswith(prefix) or Path(n[len(prefix):]).name!=n[len(prefix):] for n in names):
            raise ValueError('Unexpected target archive layout')
        def read(name):return archive.read(prefix+name)
        def get(name):return parse(read(name))
        hashes=get('source_hashes.json')
        if set(hashes)!={n[len(prefix):] for n in names if n!=prefix+'source_hashes.json'}:
            raise ValueError('Incomplete original source hash inventory')
        for name,expected in hashes.items():
            if digest(read(name))!=expected:raise ValueError('Original source file differs: '+name)
        report=get('result.json');anchor=get('task_anchor.json');target=get('latest_target.json')
        if (report.get('delivery_version')!='visual-task-binding-v4-20261020'
                or report.get('outcome')!='equal_height_plane_target_candidates_saved'
                or report.get('target_candidates',0)<1 or report.get('error') or report.get('cleanup_errors')
                or report.get('latest_target')!=target or report.get('motion_authorized') is not False
                or report.get('execution_ready') is not False):
            raise ValueError('Completed nonmotion live v4 target record required')
        for name,expected in report['runtime_source_sha256'].items():
            if Path(name).name!=name or digest((ROOT/'deployment'/name).read_bytes())!=expected:
                raise ValueError('Actual live vision source version changed: '+name)
        if len(report['runtime_source_sha256'])!=6:
            raise ValueError('Complete actual v4 runtime source inventory required')
        from visual_task_anchor import accepted_grasp,validate_saved_anchor
        grasp=accepted_grasp((ROOT/'deployment/visual_binding_grasp_source_123051.json').read_bytes(),
            (ROOT/'deployment/visual_binding_pregrasp_source_024412.json').read_bytes(),
            (ROOT/'deployment/visual_binding_pregrasp_crosscheck_081023.json').read_bytes())
        context=report['binding_camera_context']
        validate_saved_anchor(anchor,context,grasp)
        for stem in ('reference_observation','reference_image'):
            if digest(read(anchor[stem+'_file']))!=anchor[stem+'_sha256']:
                raise ValueError('Original reference evidence differs')
        if digest(read('task_anchor.json'))!=report['task_anchor_sha256'] or target['task_anchor_sha256']!=report['task_anchor_sha256']:
            raise ValueError('Target-to-reference hash differs')
        candidate=ROOT/'deployment/camera_position_candidate_225543.json'
        if digest(candidate.read_bytes())!=context['candidate_sha256']:
            raise ValueError('Frozen camera transform changed')
        if digest((ROOT/'deployment/black_cylinder_measured_20261020.json').read_bytes())!=context['shell_model_sha256']:
            raise ValueError('Measured shell model changed')
        if digest((ROOT/'src/battery_grasp/models/battery_v3_camera.torchscript').read_bytes())!=context['weights_sha256']:
            raise ValueError('Deployed camera model changed')
        plane=get('equal_height_plane.json')
        if (digest(read('equal_height_plane.json'))!=report['equal_height_plane_sha256']
                or target.get('equal_height_plane')!=plane
                or plane['original_task_anchor_sha256']!=report['task_anchor_sha256']
                or plane.get('same_upright_shell_equal_seated_top_plane_confirmed') is not True
                or plane.get('original_camera_context')!=context
                or plane.get('original_reference_observation_sha256')!=anchor['reference_observation_sha256']
                or plane['plane_point_world_m']!=anchor['reference_top_center_world_m']):
            raise ValueError('Recorded equal-height scope/source differs')
        burst=target['selected_burst']
        if (target.get('scope')!='continuous_live_ray_plane_to_fixed_wrist_pregrasp_TCP_candidate'
                or target.get('method')!='live_camera_ray_intersection_with_equal_top_height_plane'
                or target.get('fresh_burst_qualified') is not True
                or burst.get('burst_qualified') is not True or burst.get('frame_quality_pass') is not True
                or burst.get('reject_reasons') or burst.get('frame_count',0)<8
                or burst.get('elapsed_seconds',0)<2
                or any(not 0<=burst.get(k,math.inf)<=3 for k in
                    ('top_center_repeatability_max_mm','geometric_body_center_repeatability_max_mm'))
                or target.get('prepare_recipe')!=PREPARE
                or target.get('motion_authorized') is not False or target.get('execution_ready') is not False):
            raise ValueError('Qualified original plane burst and expected PREPARE required')
        observation=get(target['observation']+'.json')
        if observation.get('task_target')!=target or observation['burst_summary']!=burst:
            raise ValueError('Target is not the actual saved observation result')
        pose=vector(target['pregrasp_tcp_candidate_m_rad'],6)
        old=vector(anchor['reference_pregrasp_tcp_pose_m_rad'],6)
        point=vector(target['selected_top_center_world_m'],3)
        origin=vector(anchor['reference_top_center_world_m'],3)
        if (pose[3:]!=old[3:] or math.dist(point,origin)>.700
                or any(abs(pose[i]-old[i]-(point[i]-origin[i]))>1e-10 for i in range(3))):
            raise ValueError('Not the continuous fixed-wrist target or outside0.7m task extent')
        receipt=dict(archive_path=str(Path(path).resolve()),archive_sha256=digest(raw),
            target_sha256=digest(read('latest_target.json')),task_anchor_sha256=report['task_anchor_sha256'],
            original_internal_hashes=hashes,live_vision_version=report['delivery_version'],
            target=target,accepted_pregrasp_arm=grasp['arm'],old_fault_last_arm_state_used=False,
            absolute_accuracy_accepted=False,current_arm_or_hand_qualified=False)
        return receipt
