"""Operator offset: image-left2.8mm plus image-lower-left4.5mm, World-Z fixed."""
import hashlib
import io
import json
import math
from pathlib import Path
import zipfile

SOURCE_SHA='974667c1c6854a504fbb57e6bcf0e6c80985064ec8db4509c90c00729b6005e6'


def corrected_goal(receipt):
    target=receipt['target'];original=target['pregrasp_tcp_candidate_m_rad'][:]
    plane=target['equal_height_plane'];matrix=plane['world_from_camera']
    K=plane['original_camera_context']['K'];fx,fy=K[0][0],K[1][1]
    point=target['selected_top_center_world_m']
    if (len(matrix)!=4 or any(len(row)!=4 for row in matrix)
            or any(type(v) not in (int,float) or not math.isfinite(v) for row in matrix for v in row)
            or any(type(v) not in (int,float) or not math.isfinite(v) or v<=0 for v in (fx,fy))
            or len(point)!=3 or any(not math.isfinite(v) for v in point)):
        raise ValueError('Original finite camera geometry/intrinsics required')
    ray=[point[i]-matrix[i][3] for i in range(3)]
    if abs(ray[2])<.05:raise ValueError('Horizontal image-offset mapping ill-conditioned')
    def horizontal_step(du,dv,distance_m):
        step=[matrix[i][0]*du/fx+matrix[i][1]*dv/fy for i in range(3)]
        direction=[step[i]-ray[i]*step[2]/ray[2] for i in range(3)]
        length=math.hypot(direction[0],direction[1])
        if length<1e-8 or abs(direction[2])>1e-9:raise ValueError('Invalid image-offset direction')
        return [distance_m*direction[0]/length,distance_m*direction[1]/length,0.]
    prior=horizontal_step(-1.,0.,.0028)
    additional=horizontal_step(-1.,1.,.0045)
    total=[prior[i]+additional[i] for i in range(3)]
    goal=original[:]
    for i in range(3):goal[i]+=total[i]
    return goal,dict(scope='operator_task_offset_not_handeye_or_TCP_calibration',
        operator_direction='head_D435_live_image_left_then_lower_left',
        prior_left_mm=2.8,additional_lower_left_total_distance_mm=4.5,
        lower_left_image_direction_uv=[-1,1],
        method='pinhole_image_direction_intersection_with_constant_World_Z_at_selected_target',
        prior_delta_world_mm=[v*1000 for v in prior],
        additional_delta_world_mm=[v*1000 for v in additional],
        delta_world_mm=[v*1000 for v in total],
        original_target_pose_m_rad=original,corrected_target_pose_m_rad=goal,
        height_and_orientation_unchanged=True,object_position_or_calibration_modified=False,
        physical_alignment_accepted=False,collision_free_accepted=False)


def completed_source(path):
    # Only the recorded, cleanly completed hover segment is a candidate.
    # The enclosing run timed out at permission2; its fault last state is not used.
    raw=Path(path).read_bytes()
    if hashlib.sha256(raw).hexdigest()!=SOURCE_SHA:raise ValueError('Expected original143310 result')
    with zipfile.ZipFile(io.BytesIO(raw)) as z:
        names=z.namelist();prefix=names[0].split('/')[0]+'/'
        hashes=json.loads(z.read(prefix+'source_hashes.json'))
        if (len(names)!=len(set(names)) or z.testzip()
                or set(hashes)!={n[len(prefix):] for n in names if n!=prefix+'source_hashes.json'}
                or any(hashlib.sha256(z.read(prefix+n)).hexdigest()!=h for n,h in hashes.items())):
            raise ValueError('Original143310 hash inventory differs')
        parent=json.loads(z.read(prefix+'result.json'));worker=json.loads(z.read(prefix+'worker_result.json'))
        events=[json.loads(x) for x in z.read(prefix+'operator_events.jsonl').splitlines()]
        if (parent['worker']!=worker or len(parent['exceptions'])!=1
                or parent['exceptions'][0]['error']!="TimeoutError('Onsite permission expired')"
                or len(worker['exceptions'])!=1 or worker['exceptions'][0]['error']!="KeyboardInterrupt('Parent abort')"
                or worker['cleanup_errors'] or worker['permission_index']!=2
                or worker.get('pregrasp_endpoint_feedback_completed')
                or worker['arm_actuator_writes']!=3 or worker['hand_register_writes']!=1
                or len(worker['arm_segments'])!=3 or len(events)!=1
                or events[0]['index']!=1 or events[0]['value']!='m'
                or prefix+'permissions/2.json' in names):
            raise ValueError('Only permission2 timeout after3 completed moves is qualified')
        segments=worker['arm_segments'];last=segments[-1];arm=last['endpoint']
        if (last['phase']!='approaching_visual_hover_140mm'
                or any(not s['notification']['acknowledged'] or not s['notification']['event_seen'] for s in segments)
                or arm['arm_error']!=0 or arm['system_error']!=0):
            raise ValueError('Clean completed hover endpoint required')
        return dict(source_archive_sha256=SOURCE_SHA,
            source_role='clean_completed_hover_segment_not_fault_last_state',final_arm=arm,
            completed_hover_pose=arm['pose'][:],old_fault_last_arm_state_used=False,
            physical_alignment_accepted=False,continuation_requires_fresh_match=True)
