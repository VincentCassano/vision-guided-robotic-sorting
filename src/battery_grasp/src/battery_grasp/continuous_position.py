"""Continuous camera XYZ -> World XYZ; no slot lookup and no robot driver.

Position-only calibration never supplies commanded wrist orientation. A grasp
plan also needs a separately qualified hand-installation/TCP geometry profile.
"""
import hashlib
import json
import math
from pathlib import Path


def vector(value):
    if not isinstance(value,(list,tuple)) or len(value)!=3 or not all(type(x) in (int,float) and math.isfinite(x) for x in value):
        raise ValueError('Three finite coordinates required')
    return [float(x) for x in value]


class ContinuousPosition:
    def __init__(self,candidate_path,validation_path):
        self.path=Path(candidate_path);raw=self.path.read_bytes();self.sha256=hashlib.sha256(raw).hexdigest()
        self.candidate=json.loads(raw)
        validation_path=Path(validation_path);self.validation=json.loads(validation_path.read_text(encoding='utf8'))
        c,v=self.candidate,self.validation
        if (c.get('model')!='continuous_camera_point_to_World_with_unknown_tool_marker_center_9DOF_v1'
                or c.get('fit_position_pass') is not True or c.get('fit_count')!=16
                or c.get('excluded_samples') or c.get('position_threshold_mm')!=5
                or c.get('full6D_handeye_pass') is not False):
            raise ValueError('Reviewed position-only candidate required; do not promote failed6D calibration')
        rows=v.get('validation',[])
        if (v.get('candidate_sha256')!=self.sha256 or v.get('position_validation_pass') is not True
                or v.get('independent_validation_count')!=6 or len(rows)!=6
                or v.get('validation_used_for_fit') is not False or v.get('error')
                or v.get('source_candidate_changed') is not False):
            raise ValueError('Six independent RGBD positions must pass; no continuous grasp plan yet')
        for index,row in enumerate(rows,1):
            if (row.get('id')!='%03d'%index or row.get('candidate_sha256')!=self.sha256
                    or type(row.get('position_error_rgbd_mm')) not in (int,float)
                    or not math.isfinite(row['position_error_rgbd_mm']) or not 0<=row['position_error_rgbd_mm']<=5):
                raise ValueError('Invalid independent position record')
        hashes=json.loads((validation_path.parent/'validation_source_hashes.json').read_text(encoding='utf8'))
        for name,digest in hashes.items():
            p=(validation_path.parent/name).resolve()
            if p.parent!=validation_path.parent.resolve() or hashlib.sha256(p.read_bytes()).hexdigest()!=digest:
                raise ValueError('Validation evidence changed')
        self.matrix=c['world_from_camera'];a=self.matrix
        if (len(a)!=4 or any(len(row)!=4 for row in a)
                or not all(type(x) in (int,float) and math.isfinite(x) for row in a for x in row)
                or a[3]!=[0,0,0,1]):raise ValueError('Invalid point transform')
        for i in range(3):
            for j in range(3):
                if abs(sum(a[k][i]*a[k][j] for k in range(3))-(1 if i==j else 0))>1e-6:
                    raise ValueError('Point transform rotation not orthonormal')
        determinant=sum(a[0][i]*(a[1][(i+1)%3]*a[2][(i+2)%3]-a[1][(i+2)%3]*a[2][(i+1)%3]) for i in range(3))
        if abs(determinant-1)>1e-6:raise ValueError('Reflected point transform')

    def world_point(self,camera_point_m):
        p=vector(camera_point_m)
        if p[2]<=0:raise ValueError('Camera point must have positive depth')
        result=[sum(self.matrix[i][j]*p[j] for j in range(3))+self.matrix[i][3] for i in range(3)]
        inside=all(sum(e[j]*p[j] for j in range(3))+e[3]<=.005 for e in self.candidate['fit_camera_hull_equations'])
        return dict(point_world_m=result,source_camera_point_m=p,inside_FIT_spatial_support=inside,
            calibration_sha256=self.sha256,calibration_scope='position_only',
            mapping_kind='continuous_rigid_transform_no_slot_table',execution_ready=False)

    def grasp_plan(self,axis_grasp_center_camera_m,profile,current_hand_installation_signature):
        point=self.world_point(axis_grasp_center_camera_m)
        if not point['inside_FIT_spatial_support']:raise ValueError('Outside measured calibration region; no extrapolated grasp')
        if (profile.get('geometry_verified') is not True
                or profile.get('point_kind')!='cylinder_axis_grasp_center'
                or profile.get('hand_installation_signature')!=current_hand_installation_signature):
            raise ValueError('Actual installed-hand grasp-center/TCP relationship required; surface points are not grasp centers')
        offset=vector(profile['tcp_to_axis_grasp_center_tool_m'])
        rpy=vector(profile['fixed_robot_grasp_rpy_rad'])
        from .real_grasp import rotation
        r=rotation(rpy);world_offset=[sum(r[i][j]*offset[j] for j in range(3)) for i in range(3)]
        grasp=[point['point_world_m'][i]-world_offset[i] for i in range(3)]
        height=profile['pregrasp_vertical_offset_m']
        if type(height) not in (int,float) or not math.isfinite(height) or height<=0:raise ValueError('Measured pregrasp offset required')
        pregrasp=grasp[:];pregrasp[2]+=height
        return dict(**point,grasp_sdk_pose_m_rad=grasp+rpy,pregrasp_sdk_pose_m_rad=pregrasp+rpy,
            no_slot_rounding=True,fixed_robot_wrist_orientation=True,
            camera_marker_orientation_used_for_wrist=False,path_swept_clearance_verified=False,
            actuator_permission=False)
