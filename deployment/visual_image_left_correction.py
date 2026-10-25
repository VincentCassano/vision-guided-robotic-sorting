"""Explicit task offset in camera-image left at unchanged World height."""
import hashlib
import io
import json
import math
from pathlib import Path
import zipfile

COMPLETED_SHA='8ac04e04f1928b53770d382c91ab0aad7a5c25bfd0d49c88b204f78c953d4baf'


def corrected_goal(receipt):
    # The target/transform are unchanged source records. This is a separately
    # labelled grasp offset, not a change to calibration or detected object XYZ.
    target=receipt['target'];original=target['pregrasp_tcp_candidate_m_rad'][:]
    matrix=target['equal_height_plane']['world_from_camera']
    point=target['selected_top_center_world_m']
    if (len(matrix)!=4 or any(len(row)!=4 for row in matrix)
            or any(type(v) not in (int,float) or not math.isfinite(v) for row in matrix for v in row)
            or len(point)!=3 or any(not math.isfinite(v) for v in point)):
        raise ValueError('Finite original camera mapping/target required')
    camera_x=[matrix[i][0] for i in range(3)]
    ray=[point[i]-matrix[i][3] for i in range(3)]
    if abs(ray[2])<.05:raise ValueError('Image-left horizontal direction ill-conditioned')
    # At constant World-Z, stay in the same image-row ray plane. The negative
    # camera-X differential points left; the ray term cancels its World-Z.
    direction=[-camera_x[i]+ray[i]*camera_x[2]/ray[2] for i in range(3)]
    length=math.hypot(direction[0],direction[1])
    if length<.1 or abs(direction[2])>1e-9:raise ValueError('Invalid horizontal image-left direction')
    delta=[.0028*direction[0]/length,.0028*direction[1]/length,0.]
    goal=original[:]
    for i in range(3):goal[i]+=delta[i]
    return goal,dict(scope='same_shell_fixed_wrist_horizontal_grasp_offset_not_handeye_or_TCP_calibration',
        operator_direction='head_D435_live_image_left',distance_mm=2.8,
        method='negative_camera_X_ray_differential_intersect_constant_World_Z',
        original_target_pose_m_rad=original,corrected_target_pose_m_rad=goal,
        delta_world_mm=[v*1000 for v in delta],height_and_orientation_unchanged=True,
        object_position_or_calibration_modified=False,physical_alignment_accepted=False)


def completed_source(path):
    raw=Path(path).read_bytes()
    if hashlib.sha256(raw).hexdigest()!=COMPLETED_SHA:
        raise ValueError('Expected original 111031 completed run; no fault last-state substitute')
    def checked(archive):
        names=archive.namelist();prefix=names[0].split('/')[0]+'/'
        hashes=json.loads(archive.read(prefix+'source_hashes.json'))
        if (len(names)!=len(set(names)) or archive.testzip()
                or set(hashes)!={n[len(prefix):] for n in names if n!=prefix+'source_hashes.json'}
                or any(hashlib.sha256(archive.read(prefix+n)).hexdigest()!=h for n,h in hashes.items())):
            raise ValueError('Completed run hash inventory differs')
        return prefix,json.loads(archive.read(prefix+'result.json'))
    with zipfile.ZipFile(io.BytesIO(raw)) as outer:
        prefix,parent=checked(outer)
        nested=outer.read(prefix+'motion_result_v4.zip')
        if hashlib.sha256(nested).hexdigest()!=parent['motion_archive_sha256']:
            raise ValueError('Completed motion archive changed')
        with zipfile.ZipFile(io.BytesIO(nested)) as inner:
            pre,motion=checked(inner);worker=motion['worker']
            if (motion.get('worker_returncode')!=0 or motion.get('exceptions')
                    or worker.get('exceptions') or worker.get('cleanup_errors')
                    or worker.get('phase')!='completed'
                    or worker.get('pregrasp_endpoint_feedback_completed') is not True
                    or worker.get('grasp_performed') is not False
                    or worker.get('arm_actuator_writes')!=4 or len(worker.get('arm_segments',[]))!=4):
                raise ValueError('Clean completed descent feedback required for vertical retreat')
            final=worker['final_arm'];segment=worker['arm_segments'][-1]
            if (segment['phase']!='descending_to_visual_noncontact_pregrasp'
                    or segment['endpoint']!=final
                    or not segment['notification'].get('event_seen')
                    or final['arm_error']!=0 or final['system_error']!=0):
                raise ValueError('Actual completed final endpoint required')
            return dict(source_archive_sha256=COMPLETED_SHA,final_arm=final,
                completed_hover_pose=worker['arm_segments'][-2]['endpoint']['pose'],
                source_operator_verdict=motion['operator_endpoint_verdict'],
                physical_alignment_accepted=False,not_fault_last_state=True)
