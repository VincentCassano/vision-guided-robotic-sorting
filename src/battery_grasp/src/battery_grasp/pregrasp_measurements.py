"""Physical-reference measurement worksheet; never a motion authorizer.

No CAD or World/MDH coordinates are requested from the operator. The optional
gross sphere uses triangle inequality about an identified flange rim point.
It deliberately cannot certify the battery aperture or actual swept path.
"""
import math
from .real_grasp import number

FIELDS=(
    ('board_length_mm','板长','沿亚克力板顶面较长的外边量总长，不量裸桌。'),
    ('board_width_mm','板宽','沿亚克力板顶面较短的外边量总宽。'),
    ('board_thickness_mm','板厚','在板外缘量底面到顶面的厚度，不含下面桌面。'),
    ('battery_diameter_mm','空壳外径','量圆柱外侧最大直径，不能量标签宽或圆周长。'),
    ('battery_height_above_board_mm','空壳露出板顶高度','从亚克力板顶面垂直量到空壳最高点；凹槽内总高不能替代此值。'),
    ('battery_to_nearest_board_edge_mm','空壳到最近板边','量空壳外侧到最近板外边的最短水平间距，不是圆心距离。'),
    ('nearest_obstacle_gap_from_battery_mm','附近障碍到空壳','量料盒、另一只手/臂、机身或其他障碍外表面到空壳外侧的最短间距；没有可确认范围填u。'),
    ('nearest_obstacle_height_above_board_mm','附近障碍高度','量上述障碍最高点高出板顶多少；这只记录尺寸，不替代完整场景边界。'),
    ('flange_outer_diameter_mm','实际法兰外径','在已辨认的末端圆形安装面量其外径；不是相机前盖或手掌壳。不可见就填u，勿拆手。'),
    ('hand_max_distance_from_rim_P_mm','P到整手的最大直线距离','在同一法兰外缘选固定点P，从P到当前整手各外表面取最大直线距离，含拇指、四指及小拇指；不沿弯曲手指走尺。无法确认全外形的最大值、必须挤进夹缝或施力才量得到，就u。'),
    ('camera_adapter_max_distance_from_rim_P_mm','P到相机/支架/接头最大直线距离','仍从同一P量到腕部相机壳、支架、接头等外表面，取最大直线距离；安装在法兰后方的部件也包括。'),
    ('current_hand_to_board_gap_mm','当前整手到板顶的最小间隙','只量当前整手最低外表面到亚克力板顶面的垂直间隙；不要自动填旧14mm。最低点不在实际板面垂直投影内、或不能安全量，就u。'),
    ('current_hand_to_battery_gap_mm','当前整手到空壳最小间隙','量当前整手任何外表面与空壳外侧的最短距离，不只量小拇指或垂直高度；不能安全量就u。'),
    ('current_hand_to_other_obstacle_gap_mm','当前整手到其他障碍最小间隙','量当前整手与料盒、另一臂/手、机身等最近的外表面间距。'),
    ('thumb_finger_entry_width_mm','当前虎口入口净宽','旁侧对照刻度，记录拇指内侧到对面四指内侧的最小入口宽度；不把尺子塞进夹缝，不要求动手指。'),
    ('measurement_allowance_mm','每笔长度测量可能误差上限','填写每笔卷尺读数、参考点定位和重复摆尺的可能误差上限e，mm，必须>0；程序按各公式累计误差。不清楚填u，不能默认0。'),
)
KEYS=tuple(row[0] for row in FIELDS)
ZERO_ALLOWED=('battery_to_nearest_board_edge_mm','nearest_obstacle_gap_from_battery_mm',
    'current_hand_to_board_gap_mm','current_hand_to_battery_gap_mm','current_hand_to_other_obstacle_gap_mm',
    'thumb_finger_entry_width_mm')
OBSERVATIONS=('holding_state','flange_rim_point_identified','reference_description',
              'all_visible_moving_cables_listed','arm_hand_unchanged_during_measurement')


def parse_length(value,label='length',allow_zero=False):
    if value is None or isinstance(value,str) and value.strip().lower() in ('u','unknown','未知',''):
        return None
    if isinstance(value,str):
        try:value=float(value.strip())
        except ValueError:raise ValueError(label+': enter millimetres, decimals allowed, oru')
    value=number(value,label)
    if value<0 or value==0 and not allow_zero:raise ValueError(label+': positive millimetres required; unknown is not0')
    return value


def validate_worksheet(data):
    if not isinstance(data,dict) or type(data.get('schema_version')) is not int or data['schema_version']!=1:
        raise ValueError('Measurement schema1 required')
    values=data.get('lengths',{})
    if not isinstance(values,dict):raise ValueError('Length object required')
    if set(values)-set(KEYS):raise ValueError('Unknown measurement key')
    lengths={key:parse_length(values.get(key),key,allow_zero=key in ZERO_ALLOWED) for key in KEYS}
    o=data.get('observations',{})
    if not isinstance(o,dict) or set(o)-set(OBSERVATIONS):raise ValueError('Invalid observation fields')
    if o.get('holding_state','unknown') not in ('empty','object','unknown'):raise ValueError('Invalid holding state')
    observations={key:o.get(key) for key in OBSERVATIONS}
    observations['holding_state']=o.get('holding_state','unknown')
    for key in ('flange_rim_point_identified','all_visible_moving_cables_listed','arm_hand_unchanged_during_measurement'):
        if observations[key] is not None and type(observations[key]) is not bool:raise ValueError('Observation must be bool or unknown')
    if observations['reference_description'] is not None and not isinstance(observations['reference_description'],str):
        raise ValueError('Reference description must be text')
    if isinstance(observations['reference_description'],str) and observations['reference_description'].strip().lower() in ('u','unknown','未知',''):
        observations['reference_description']=None
    cables=[]
    for row in data.get('cables',[]):
        if not isinstance(row,dict) or set(row)-{'id','anchor_A_follows_end_assembly','rim_P_to_anchor_A_mm','free_length_A_to_B_mm','note'}:
            raise ValueError('Invalid cable record')
        if not isinstance(row.get('id'),str) or not row['id'].strip():raise ValueError('Cable identifier required')
        attached=row.get('anchor_A_follows_end_assembly')
        if attached is not None and type(attached) is not bool:raise ValueError('Cable attachment must be bool or unknown')
        cables.append(dict(id=row['id'],anchor_A_follows_end_assembly=attached,
            rim_P_to_anchor_A_mm=parse_length(row.get('rim_P_to_anchor_A_mm'),'P-to-A'),
            free_length_A_to_B_mm=parse_length(row.get('free_length_A_to_B_mm'),'free cable length'),note=row.get('note')))
    if len({row['id'] for row in cables})!=len(cables):raise ValueError('Duplicate cable ids')
    return dict(schema_version=1,lengths=lengths,observations=observations,cables=cables)


def zero_flange_frame(worker):
    try:
        signature=worker['analysis']['current_arm']['frame_signatures']['tool']
        return (signature['name']=='Arm_Tip' and
            all(abs(number(v,'frame'))<=1e-9 for group in ('position','euler') for v in signature[group].values()) and
            len(signature['position'])==len(signature['euler'])==3)
    except (KeyError,TypeError,ValueError):return False


def summarize_measurements(data,before=None,after=None):
    w=validate_worksheet(data);l=w['lengths'];o=w['observations']
    result=dict(schema_version=1,read_only=True,execution_ready=False,hardware_writes=0,
        arm_motion_allowed=False,hand_motion_allowed=False,automatic_return=False,
        measurements=w,unknown_lengths=[key for key,value in l.items() if value is None],
        historical_reference=dict(lowest_hand_outer_surface_to_acrylic_board_top_mm=14,
            current_measurement=False,applies_to='historical_PREPARE_endpoint_only'),
        fresh_static_brackets_recorded=False,manual_measurement_pose_bracket_consistent=False,
        current_hand_shape='unknown',fixed_PREPARE_geometry_recorded=False,
        flange_to_Arm_Tip_mapping_physically_verified=False,installed_geometry_physically_verified=False,
        gross_hand_sphere_candidate=None,cable_sphere_candidates=[],aperture_fit_verified=False,
        whole_scene_geometry_verified=False,controller_path_verified=False,
        missing_for_actual_descent=[],observations=[])
    valid=bool(before and after and before.get('static_readout_valid') is True and after.get('static_readout_valid') is True)
    result['fresh_static_brackets_recorded']=valid
    if valid:
        a,b=before['analysis']['current_arm'],after['analysis']['current_arm']
        first,last=before['hand_preflight_after'],after['hand_preflight_after']
        same=(a['frame_signatures']==b['frame_signatures'] and a['limits']==b['limits'] and
            before['configuration']==after['configuration'] and first['speeds']==last['speeds'] and
            first['force_targets']==last['force_targets'] and
            math.dist(a['tcp_position_mm'],b['tcp_position_mm'])<=.5 and
            all(abs(x-y)<=.2 for x,y in zip(a['joint_degrees'],b['joint_degrees'])) and
            all(abs(math.atan2(math.sin(x-y),math.cos(x-y)))<=.005 for x,y in zip(a['tcp_rpy_rad'],b['tcp_rpy_rad'])) and
            all(abs(x-y)<=2 for x,y in zip(first['targets'],last['targets'])) and
            all(abs(x-y)<=150 for x,y in zip(first['positions'],last['positions'])))
        result['manual_measurement_pose_bracket_consistent']=same
        result['current_arm']=b
        result['current_hand_shape']=after['analysis']['hand_pose_classification'] if same else 'changed_during_measurement'
        if not same:result['observations'].append('Arm/hand changed while measuring; pose-specific dimensions cannot be attached to one state')
    else:result['observations'].append('Fresh static device bracket unavailable; physical dimensions retained without a pose mapping')
    if o['holding_state']!='empty':result['observations'].append('Empty hand not reported; do not interpret hand/opening measurements as an empty PREPARE envelope')
    result['fixed_PREPARE_geometry_recorded']=bool(valid and result['manual_measurement_pose_bracket_consistent']
        and result['current_hand_shape']=='prepare' and o['holding_state']=='empty'
        and before.get('geometry_reference_eligible') is True and after.get('geometry_reference_eligible') is True
        and o['arm_hand_unchanged_during_measurement'] is True)
    # A sphere is only a coarse numerical candidate, including measurement error.
    # It cannot preserve the empty region between thumb and fingers.
    contact=any(l[key]==0 for key in ('current_hand_to_board_gap_mm','current_hand_to_battery_gap_mm','current_hand_to_other_obstacle_gap_mm'))
    ref_ok=bool(not contact and valid and result['manual_measurement_pose_bracket_consistent'] and zero_flange_frame(before)
        and zero_flange_frame(after) and o['flange_rim_point_identified'] is True
        and o['reference_description'] and o['holding_state']=='empty'
        and o['arm_hand_unchanged_during_measurement'] is True
        and before.get('geometry_reference_eligible') is True and after.get('geometry_reference_eligible') is True)
    keys=('flange_outer_diameter_mm','hand_max_distance_from_rim_P_mm',
          'camera_adapter_max_distance_from_rim_P_mm','measurement_allowance_mm')
    if ref_ok and all(l[key] is not None for key in keys):
        radius=l['flange_outer_diameter_mm']/2+max(l[keys[1]],l[keys[2]])+1.5*l[keys[3]]
        result['gross_hand_sphere_candidate']=dict(radius_mm=radius,center_reference='candidate_zero_Arm_Tip_flange_center',
            source='operator_P_to_outer_surface_measurements_plus_rim_radius_and_explicit_allowance',
            formula='R = flange_D/2 + max(P_to_hand,P_to_camera_adapter) +1.5e; e bounds each length measurement',
            pose_scope=result['current_hand_shape'],covers_all_hand_transitions=False,
            mapping_physically_verified=False,full_shape_coverage_physically_verified=False,
            preserves_battery_aperture=False,needs_refinement_near_battery=True,execution_ready=False)
    for row in w['cables']:
        if ref_ok and row['anchor_A_follows_end_assembly'] is True and l['flange_outer_diameter_mm'] is not None and l['measurement_allowance_mm'] is not None and all(row[key] is not None for key in ('rim_P_to_anchor_A_mm','free_length_A_to_B_mm')):
            result['cable_sphere_candidates'].append(dict(id=row['id'],
                radius_mm=l['flange_outer_diameter_mm']/2+row['rim_P_to_anchor_A_mm']+row['free_length_A_to_B_mm']+2.5*l['measurement_allowance_mm'],
                source='triangle inequality from moving end anchor and full free arc length',
                mapping_physically_verified=False,coverage='supplied_free_segment_A_to_B_only',execution_ready=False))
    if l['thumb_finger_entry_width_mm'] is not None and l['battery_diameter_mm'] is not None:
        result['one_cross_section_width_minus_battery_diameter_mm']=l['thumb_finger_entry_width_mm']-l['battery_diameter_mm']
        result['observations'].append('One opening width is not a3D cavity/offset or proof that the battery fits along the full descent')
    result['operator_reported_current_external_contact']=contact
    if result['operator_reported_current_external_contact']:
        result['observations'].append('Operator reported zero current clearance: use onsite hardware stop; no automatic release/motion requested')
    gaps=[]
    if not result['fixed_PREPARE_geometry_recorded']:gaps.append('Empty fixed PREPARE shape not tied to unchanged fresh arm/hand brackets; no shape change requested by this program')
    gaps.extend(('Installed exact/detailed hand aperture and flange reference mapping, including little finger',
        'Actual arm-link, wrist-camera/adapter, all moving cable and other-arm/body envelopes',
        'Workboard/battery/obstacle position/orientation and hidden geometry; lengths are not World coordinates',
        'Physical error budgets, actual controller interpolation and fresh execution preflight'))
    if result['unknown_lengths']:gaps.append('Missing physical measurements: '+', '.join(result['unknown_lengths']))
    result['missing_for_actual_descent']=gaps
    result['outcome']='physical_measurements_collected_descent_pending'
    return result
