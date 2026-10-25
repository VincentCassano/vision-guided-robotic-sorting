"""Relative plane clearance and arm-axis motion from accepted onsite numerics.

No SDK, network or executor. A14mm hand-to-reference-plane anchor is not a
battery gap. Conditional rigid-hand plane mathematics cannot certify a scene.
"""
import hashlib
import json
import math
from .real_grasp import number,rotation
from .taught_dh_model import EXPECTED_DH
from .taught_path import reserves

NATIVE_REPORT_SHA='1b3500c3d52651f435cd98e24e17cbcba9ad3fe576d5ce0b59d75c7f9de23c2d'
OPTIONAL_ALLOWANCES=('raised_surface_above_reference_mm','measurement_allowance_mm',
    'relative_model_allowance_mm','tracking_position_allowance_mm',
    'tracking_rotation_allowance_degrees','whole_hand_radius_about_arm_tip_mm')


def accepted_native_result(raw):
    if hashlib.sha256(raw).hexdigest()!=NATIVE_REPORT_SHA:raise ValueError('Only audited200357 onsite numerical result accepted')
    d=json.loads(raw);n=d['native_assessment']
    if (d['outcome']!='both_numerical_candidates_calculated_geometry_pending' or d['exceptions']
            or d['execution_ready'] is not False or d['hardware_writes']!=0
            or d['network_connection_requested'] is not False or d['robot_motion_allowed'] is not False
            or d['hand_motion_allowed'] is not False or n['worker_completed'] is not True
            or n['native_model_agreement'] is not True or n['joint_candidate_calculated'] is not True
            or n['vertical_candidate_calculated'] is not True or n['exceptions']
            or n['execution_ready'] is not False or n['hardware_writes']!=0
            or n['network_connection_requested'] is not False or n['robot_instance_constructed'] is not False
            or len(n['joint_path'])!=84 or len(n['vertical_path'])!=44):
        raise ValueError('Onsite numerical provenance/scope differs')
    return d


def rotation_difference_degrees(a,b):
    ra,rb=rotation(a),rotation(b)
    cosine=(sum(ra[i][j]*rb[i][j] for i in range(3) for j in range(3))-1)/2
    return math.degrees(math.acos(max(-1,min(1,cosine))))


def axis_frames(joints):
    """Installed actual-DH axis centres, not physical arm/cable envelopes."""
    if not isinstance(joints,(list,tuple)) or len(joints)!=6:raise ValueError('Six axis angles required')
    joints=[number(v,'axis angle') for v in joints]
    b=math.pi/4
    transform=[[math.cos(b),0,math.sin(b),0],[0,1,0,0],[-math.sin(b),0,math.cos(b),0],[0,0,0,1]]
    result=[]
    for joint,row in zip(joints,EXPECTED_DH):
        alpha,a,d,offset=math.radians(row[0]/1000),row[1]/1e6,row[2]/1e6,row[3]/1000
        theta=math.radians(joint+offset);c,s,ca,sa=math.cos(theta),math.sin(theta),math.cos(alpha),math.sin(alpha)
        link=[[c,-s,0,a],[s*ca,c*ca,-sa,-sa*d],[s*sa,c*sa,ca,ca*d],[0,0,0,1]]
        transform=[[sum(transform[i][k]*link[k][j] for k in range(4)) for j in range(4)] for i in range(4)]
        result.append([transform[i][3]*1000 for i in range(3)])
    return result


def input_profile(profile):
    if profile is None:profile={}
    if not isinstance(profile,dict):raise ValueError('Corridor profile must be an object')
    if profile.get('schema_version',1)!=1:raise ValueError('Unsupported corridor schema')
    if profile.get('reference_surface_kind') not in (None,'workboard_top','bare_table'):
        raise ValueError('Reference surface must be workboard_top, bare_table or unknown')
    result={key:None for key in OPTIONAL_ALLOWANCES}
    for key in result:
        if profile.get(key) is not None:
            value=number(profile[key],key)
            if value<0:raise ValueError('Negative allowance forbidden: '+key)
            if key=='whole_hand_radius_about_arm_tip_mm' and value<=0:raise ValueError('Positive whole hand radius required')
            if key=='tracking_rotation_allowance_degrees' and value>180:raise ValueError('Rotation allowance must be at most180degrees')
            result[key]=value
    result['reference_surface_kind']=profile.get('reference_surface_kind')
    result['reference_surface_source']=profile.get('reference_surface_source')
    if result['reference_surface_kind'] is not None and not result['reference_surface_source']:
        raise ValueError('Known reference surface requires its evidence source')
    result['physical_setup_reported_unchanged']=profile.get('physical_setup_reported_unchanged')
    result['physical_setup_report_source']=profile.get('physical_setup_report_source')
    if result['physical_setup_reported_unchanged'] is not None and type(result['physical_setup_reported_unchanged']) is not bool:
        raise ValueError('Invalid setup report')
    if result['physical_setup_reported_unchanged'] is not None and not result['physical_setup_report_source']:
        raise ValueError('Setup report requires its source')
    result['minimum_required_clearance_mm']=number(profile.get('minimum_required_clearance_mm',10),'required plane margin')
    if result['minimum_required_clearance_mm']<=0:raise ValueError('Positive plane margin required')
    return result


def analyse_vertical_corridor(raw,profile=None):
    source=accepted_native_result(raw);p=input_profile(profile);n=source['native_assessment'];c=source['candidate']
    rows=n['vertical_path'];anchor=rows[-1];bottom=anchor['desired_world_mm'];anchor_pose=anchor['calculated']
    if c['pregrasp_clearance_interpretation']['lowest_hand_outer_surface_to_table_mm']!=14:
        raise ValueError('Reviewed14mm reference-plane anchor differs')
    if c['pregrasp_clearance_interpretation']['hand_to_battery_clearance_mm'] is not None:
        raise ValueError('Table anchor cannot become a battery gap')
    result=dict(schema_version=1,scope='fixed_PREPARE_vertical_relative_plane_analysis_only',
        source_native_report_sha256=NATIVE_REPORT_SHA,onsite_numerical_stage_accepted=True,
        execution_ready=False,arm_motion_allowed=False,hand_motion_allowed=False,network_connection_requested=False,
        hardware_writes=0,table_world_z_derived=False,reference_anchor_gap_mm=14,
        anchor_gap_scope=('historical_PREPARE_lowest_hand_outer_surface_to_workboard_top_only'
            if p['reference_surface_kind']=='workboard_top' else 'historical_lowest_hand_outer_surface_to_measured_table_reference_only'),
        anchor_is_historical=True,current_start_known=False,current_pose_matches_anchor=False,
        current_hand_pose_known=False,current_hold_state_known=False,current_controller_configuration_verified=False,
        automatically_return_to_historical_anchor=False,rigid_plane_physical_conditions_confirmed=False,
        hand_target=[8000,8000,8000,8000,65535,65535],profile=p,rows=[],axis_samples=[],axis_motion_bounds=[],
        rigid_plane_conditions=['Unchanged installed hand and PREPARE shape',
            'Same horizontal reference plane throughout the swept area',
            'Exact vertical translation at fixed orientation; model/actual deviations budgeted separately'],
        ideal_plane_formula='14mm + desired_TCP.Z - taught_pregrasp_desired_TCP.Z',
        arm_axes_are_body_collision_envelopes=False,battery_clearance_verified=False,
        obstacles_clearance_verified=False,controller_interpolation_verified=False,
        actual_controller_path_verified=False,assumption_confirmation_grants_execution=False,
        joint_route_can_use_same_14mm_plane_certificate=False,
        blockers=['Battery, raised workboard edges and other obstacles require swept geometry',
            'Arm body and cable envelopes are unknown; joint-axis centres are not body shapes',
            'High aligned fixed-wrist pose and transition are not taught/physically accepted',
            'Controller actual interpolation, tracking bounds and fresh preflight are unverified',
            'Another project changed arm/hand poses; historical anchor is not current start, no automatic return'])
    full_budget=all(p[key] is not None for key in OPTIONAL_ALLOWANCES)
    if not full_budget:result['blockers'].append('Whole-hand radius and complete allowance budget unknown; no plane lower-bound claim')
    if p['reference_surface_kind'] is None:result['blockers'].append('The14mm reference surface is not yet identified as workboard top or bare table')
    for index,row in enumerate(rows):
        if any(abs(row['desired_world_mm'][i]-bottom[i])>1e-9 for i in (0,1)):
            raise ValueError('Onsite candidate is not fixed-XY')
        if row['desired_rpy_rad']!=anchor['desired_rpy_rad']:raise ValueError('Desired wrist is not fixed')
        if index and row['desired_world_mm'][2]>=rows[index-1]['desired_world_mm'][2]:
            raise ValueError('Candidate height is not strictly descending')
        normal=min(reserves(row['joint_degrees'],c['ordinary_limits']));drive=min(reserves(row['joint_degrees'],c['drive_limits']))
        if min(normal,drive)<3:raise ValueError('Onsite candidate joint reserve below3degrees')
        delta_z=row['desired_world_mm'][2]-bottom[2]
        model_delta_z=row['calculated']['tool_world_mm'][2]-anchor_pose['tool_world_mm'][2]
        angle=rotation_difference_degrees(row['calculated']['tool_rpy_rad'],anchor_pose['tool_rpy_rad'])
        entry=dict(index=index,desired_world_mm=row['desired_world_mm'],calculated=row['calculated'],
            joint_degrees=row['joint_degrees'],ideal_reference_plane_gap_mm=14+delta_z,
            relative_FK_z_reference_gap_mm=14+model_delta_z,rotation_from_anchor_degrees=angle,
            ordinary_minimum_reserve_degrees=normal,drive_minimum_reserve_degrees=drive,
            budgeted_plane_lower_bound_mm=None,executable=False)
        if full_budget:
            displacement=2*p['whole_hand_radius_about_arm_tip_mm']*math.sin(math.radians(min(180,angle+p['tracking_rotation_allowance_degrees']))/2)
            entry['budgeted_plane_lower_bound_mm']=14+model_delta_z-displacement-sum(p[key] for key in (
                'raised_surface_above_reference_mm','measurement_allowance_mm','relative_model_allowance_mm','tracking_position_allowance_mm'))
        result['rows'].append(entry)
        for joint,position in enumerate(axis_frames(row['joint_degrees']),1):
            result['axis_samples'].append(dict(path_index=index,joint_axis=joint,world_mm=position,body_geometry_known=False))
    for joint in range(1,7):
        positions=[row['world_mm'] for row in result['axis_samples'] if row['joint_axis']==joint]
        result['axis_motion_bounds'].append(dict(joint_axis=joint,min_mm=[min(v[i] for v in positions) for i in range(3)],
            max_mm=[max(v[i] for v in positions) for i in range(3)],
            maximum_distance_from_pregrasp_axis_mm=max(math.dist(v,positions[-1]) for v in positions),
            mesh_or_cable_radius_mm=None,physical_collision_check_pass=None))
    result.update(ideal_table_reference_analysis_complete=True,
        minimum_ideal_reference_plane_gap_mm=min(row['ideal_reference_plane_gap_mm'] for row in result['rows']),
        maximum_ideal_reference_plane_gap_mm=max(row['ideal_reference_plane_gap_mm'] for row in result['rows']),
        maximum_FK_rotation_from_anchor_degrees=max(row['rotation_from_anchor_degrees'] for row in result['rows']),
        complete_plane_allowance_budget_supplied=full_budget,budgeted_plane_scope_condition_pass=None,
        supplied_budget_numerical_pass=None,
        whole_scene_collision_ready=False)
    if full_budget:
        result['supplied_budget_numerical_pass']=min(row['budgeted_plane_lower_bound_mm'] for row in result['rows'])>=p['minimum_required_clearance_mm']
    return result
