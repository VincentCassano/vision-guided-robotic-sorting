"""Fresh visual cycle at the measured 183637 upper pose, without old-point alignment.

The recorded 25.3 +/- 0.1 mm gap is an operator measurement, not a model-derived
collision certificate. This mode returns empty to its freshly accepted start;
it never authorizes lateral transfer or interprets a stop ACK as physical stop.
"""
import copy
import hashlib
import json
import math
from pathlib import Path

from battery_grasp.localized_pregrasp import select_surface
from battery_grasp.pregrasp_replay import check_segment
from battery_grasp.real_grasp import orientation_angle_rad
from high_descent_prepare_worker import same_pose
from return_after_release_worker import (
    SAFETY_CONFIGURATION, arm_shape, metadata_signature, model_signature,
    strict_json)
from vision_clearance_return_cycle import _integer, _number, _prepare_profile
from vision_cycle_hand import PREPARE
from vision_cycle_sequence import CALIBRATION_SHA
from vision_cycle_supported_release import SupportedCycle


SOURCE_STEM = 'hand_clearance_return_20261016_183637_927921'
SOURCE_ZIP_SHA256 = '1032ce859b952099e792b8e0a61e1fc75e4224ab869529d34a61627154f2f23f'
SOURCE_PAYLOAD_SHA256 = {
    'worker': 'e404b0a5c49fc051022d4b2165e15e73f3334517bca0d71099c11249457d3457',
    'parent': '619fa508bf7298f921907226860a9c663f09ceab762a3a805b81832bfcc2e900'}
MEASUREMENT_BASIS = 'vertical_lowest_whole_hand_surface_above_current_battery_top_horizontal_plane'
SCENE_FLAG = 'same_high_return_shell_scene_and_clearance_confirmed'


def _captured_bytes(folder, record, role):
    name = 'confirmation.json' if role == 'confirmation' else role + '_result.json'
    relative = 'high_return_source/' + name
    raw = (folder / relative).read_bytes()
    if (type(record.get('byte_count')) is not int
            or record['byte_count'] != len(raw)
            or record.get('captured_path') != relative
            or record.get('sha256') != hashlib.sha256(raw).hexdigest()):
        raise ValueError('Captured measured-upper source bytes changed: ' + role)
    if role in SOURCE_PAYLOAD_SHA256 and hashlib.sha256(raw).hexdigest() != SOURCE_PAYLOAD_SHA256[role]:
        raise ValueError('Only the reviewed 183637 source is accepted: ' + role)
    return raw


def read_high_return_context(folder, profile):
    """Accept exact field evidence plus the separately recorded user measurement."""
    current_parent = strict_json((folder / 'parent_result.json').read_bytes())
    records = current_parent['high_return_source']
    old = strict_json(_captured_bytes(folder, records['worker_report'], 'worker'))
    parent = strict_json(_captured_bytes(folder, records['parent_report'], 'parent'))
    confirmation_raw = _captured_bytes(folder, records['confirmation'], 'confirmation')
    bundled_confirmation = Path(__file__).with_name('clearance_183637_operator_confirmation.json').read_bytes()
    if confirmation_raw != bundled_confirmation:
        raise ValueError('Captured clearance confirmation differs from the reviewed delivery')
    confirmation = strict_json(confirmation_raw)
    _integer(confirmation.get('schema_version'), 1, 'confirmation.schema_version')
    if (confirmation.get('source_zip') != SOURCE_STEM + '.zip'
            or confirmation.get('source_zip_sha256') != SOURCE_ZIP_SHA256
            or confirmation.get('source_worker_sha256') != SOURCE_PAYLOAD_SHA256['worker']
            or confirmation.get('source_parent_sha256') != SOURCE_PAYLOAD_SHA256['parent']
            or confirmation.get('measurement_basis') != MEASUREMENT_BASIS
            or confirmation.get('lateral_transfer_ready') is not False
            or confirmation.get('user_statement') != '上升已完成，现在手最低点距离电池顶部25.3mm左右浮动0.1mm'
            or _number(confirmation.get('measured_clearance_mm'), 'measured_clearance_mm') != 25.3
            or _number(confirmation.get('variation_mm'), 'variation_mm') != .1
            or _number(confirmation.get('minimum_reported_clearance_mm'), 'minimum_reported_clearance_mm') != 25.2):
        raise ValueError('Actual user statement/source/top-clearance measurement differs')
    for document in (old, parent):
        _integer(document.get('schema_version'), 1, 'source.schema_version')
        if (document.get('scope') != 'empty_vertical_clearance_from_completed_taught_cycle'
                or document.get('outcome') != 'aborted_no_further_writes'):
            raise ValueError('Expected the reviewed clearance-wait refusal source')
    _integer(parent.get('worker_return_code'), 1, 'parent.worker_return_code')
    if (parent.get('worker_result') != old
            or parent.get('worker_cleanup_timeout') is not False
            or parent.get('exceptions') != [{'error': "RuntimeError('Worker changed/aborted during input; stale permission refused')"}]
            or old.get('phase') != 'fault' or old.get('cleanup_errors') != []
            or len(old.get('exceptions', [])) != 1
            or old['exceptions'][0].get('error') != "RuntimeError('Unexpected arm movement during cycle wait')"
            or "wait_input(11, 'await_hand_top_clearance'" not in old['exceptions'][0].get('traceback', '')
            or old.get('permission_index') != 11
            or old.get('arm_stop_attempt') != {'result': 0, 'physical_stop_verified': False, 'hand_stop_or_release': False}
            or old.get('arm_command_attempted') is not True
            or old.get('clearance_source_context_accepted') is not True
            or old.get('clearance_source_accepted') is not True
            or old.get('clearance_return_requested') is not True
            or old.get('no_hand_or_configuration_writes') is not True
            or old.get('automatic_lateral_motion') is not False
            or old.get('load_lift_or_transfer_performed') is not False
            or old.get('hand_shape') != 'PREPARE'):
        raise ValueError('Source is not the reviewed lift-feedback / clearance-input refusal')
    for key, expected in (('arm_actuator_writes', 1), ('hand_register_writes', 0),
            ('controller_configuration_writes', 0), ('hand_bus_configuration_writes', 0)):
        _integer(old.get(key), expected, key)
    events = old.get('operator_inputs')
    if (not isinstance(events, list) or len(events) != 1 or parent.get('operator_inputs') != events
            or events[0].get('index') != 10 or events[0].get('phase') != 'await_clearance_lift'
            or events[0].get('value') != 'u'
            or events[0].get('empty_hand_upward_clearance_sweep_confirmed') is not True):
        raise ValueError('Exactly one original empty-upward-sweep permission required')
    goal, initial, final = old['taught_goal'], old['initial_arm'], old['final_arm']
    for arm in (goal, initial, final):
        arm_shape(arm)
        if arm['limits'] != goal['limits'] or arm['frame_signatures'] != goal['frame_signatures']:
            raise ValueError('Original clearance frames/limits changed')
    if (abs((final['pose'][2] - goal['pose'][2]) * 1000 - 100) > .5
            or math.dist(final['pose'][:2], goal['pose'][:2]) > .0005
            or orientation_angle_rad(final['pose'][3:], goal['pose'][3:]) > .005
            or old.get('upper') != final):
        raise ValueError('Reviewed upper is not the observed same-XY/wrist goal+100mm')
    segments = old.get('arm_segments')
    if not isinstance(segments, list) or len(segments) != 1 or segments[0].get('phase') != 'lifting_empty_clearance':
        raise ValueError('Exactly one original empty clearance segment required')
    segment = segments[0]
    start, endpoint = segment['plan']['start'], segment['endpoint']
    arm_shape(start)
    arm_shape(endpoint)
    target = segment['plan']['target_sdk_pose_m_rad']
    if (not isinstance(target, list) or len(target) != 6
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in target)
            or not same_pose(initial, start) or initial['limits'] != start['limits']
            or not same_pose(endpoint, final) or endpoint['limits'] != final['limits']
            or target[:2] != start['pose'][:2] or target[3:] != start['pose'][3:]
            or abs(target[2] - goal['pose'][2] - .100) > 1e-9
            or abs(_number(segment['plan'].get('distance_mm'), 'distance_mm')
                - (target[2] - start['pose'][2]) * 1000) > 1e-6
            or math.dist(endpoint['pose'][:3], target[:3]) > .0005
            or orientation_angle_rad(endpoint['pose'][3:], target[3:]) > .005):
        raise ValueError('Recorded single lift target/feedback differs from the reviewed corridor')
    last = old['last_arm_state']
    raw = last.get('raw')
    if (not isinstance(raw, list) or len(raw) != 5 or type(raw[0]) is not int or raw[0] != 0
            or raw[1] != last.get('joints') or raw[2] != last.get('pose')
            or type(raw[3]) is not int or raw[3] != 0
            or type(raw[4]) is not int or raw[4] != 0):
        raise ValueError('Complete final raw arm feedback with no controller errors required')
    # Retain the rejected wait sample as evidence, never as the qualified start.
    # The completed lift endpoint and its successful full preflight were already
    # checked together above; final is that accepted post-lift reference.
    observed = copy.deepcopy(final)
    observed.update(joints=copy.deepcopy(last['joints']), pose=copy.deepcopy(last['pose']),
        arm_error=raw[3], system_error=raw[4])
    observed['reserve'] = [min(q - low, high - q) for q, low, high in zip(
        observed['joints'], observed['limits']['minimum'], observed['limits']['maximum'])]
    arm_shape(observed)
    if (math.dist(observed['pose'][:3], final['pose'][:3]) > .0005
            or orientation_angle_rad(observed['pose'][3:], final['pose'][3:]) > .005):
        raise ValueError('Actual last observation is outside the measured-upper Cartesian neighborhood')
    if old['configuration'] != profile['observed_configuration']:
        raise ValueError('Source hand configuration differs from the measured profile')
    _prepare_profile(old['last_preflight'], profile)
    _prepare_profile(old['final_hand'], profile)
    metadata_signature(old['sdk_environment'])
    model_signature(old['fresh_model'])
    return dict(worker=old, parent=parent, source_records=records,
        confirmation=confirmation, last_observed_arm=observed,
        accepted_upper=copy.deepcopy(final),
        accepted_upper_endpoint=copy.deepcopy(endpoint),
        accepted_upper_source='successful_post_lift_full_preflight_final_arm',
        last_observed_arm_is_not_verified_stationary=True)


def pose_comparison(actual,expected):
    """Explain the existing pose gate without adding or widening tolerances."""
    joint_delta=[a-b for a,b in zip(actual['joints'],expected['joints'])]
    xyz_delta=[(a-b)*1000 for a,b in zip(actual['pose'][:3],expected['pose'][:3])]
    rotation=orientation_angle_rad(actual['pose'][3:],expected['pose'][3:])
    return dict(matched=same_pose(actual,expected),joint_delta_degrees=joint_delta,
        max_joint_delta_degrees=max(abs(v) for v in joint_delta),joint_limit_degrees=.08,
        xyz_delta_mm=xyz_delta,xyz_distance_mm=math.dist(actual['pose'][:3],expected['pose'][:3])*1000,
        xyz_limit_mm=.5,physical_rotation_rad=rotation,rotation_limit_rad=.005,
        frame_signatures_equal=actual['frame_signatures']==expected['frame_signatures'],
        ordinary_minimum_equal=actual['limits']['minimum']==expected['limits']['minimum'],
        ordinary_maximum_equal=actual['limits']['maximum']==expected['limits']['maximum'],
        full_limits_equal=actual['limits']==expected['limits'])


def verify_bound_descent(sequence, end, upper, mapped, plan):
    """Verify the observed descent against its actual command and live branch.

    The earlier taught wrist is provenance. MoveL holds the freshly read start
    wrist; checking that command against a different historical wrist after the
    descent would reject a motion which followed its qualified command.
    """
    for arm in (end, upper, plan['start']):
        arm_shape(arm)
    start = plan['start']
    target = plan['target_sdk_pose_m_rad']
    branch_error = None
    try:
        check_segment(end, plan)
    except (ValueError, RuntimeError) as exc:
        branch_error = repr(exc)
    upper_position = math.dist(start['pose'][:3], upper['pose'][:3])
    upper_rotation = orientation_angle_rad(start['pose'][3:], upper['pose'][3:])
    target_position = math.dist(end['pose'][:3], target[:3])
    target_rotation = orientation_angle_rad(end['pose'][3:], target[3:])
    checks = dict(
        command_preserves_fresh_start_XY=target[:2] == start['pose'][:2],
        command_preserves_fresh_start_wrist=target[3:] == start['pose'][3:],
        command_uses_bound_goal_Z=abs(target[2] - mapped[2] / 1000) <= 1e-9,
        plan_frames_match_qualified_upper=start['frame_signatures'] == upper['frame_signatures'],
        plan_full_limits_match_qualified_upper=start['limits'] == upper['limits'],
        fresh_plan_start_within_qualified_upper_xyz=upper_position <= .0005,
        fresh_plan_start_within_qualified_upper_rotation=upper_rotation <= .005,
        observed_terminal_matches_command_xyz=target_position <= .001,
        observed_terminal_matches_command_rotation=target_rotation <= .005,
        observed_live_branch_and_reserves=branch_error is None)
    detail = dict(checks=checks, failed_checks=[name for name, passed in checks.items() if not passed],
        acceptance_reference='this_descent_actual_MoveL_target_and_computed_live_branch',
        target_sdk_pose_m_rad=copy.deepcopy(target),
        endpoint_xyz_delta_from_command_mm=[(a-b)*1000 for a,b in zip(end['pose'][:3],target[:3])],
        endpoint_xyz_error_mm=target_position*1000, endpoint_xyz_limit_mm=1.,
        endpoint_physical_rotation_from_command_rad=target_rotation, rotation_limit_rad=.005,
        plan_start_xyz_error_from_qualified_upper_mm=upper_position*1000,
        plan_start_physical_rotation_from_qualified_upper_rad=upper_rotation,
        qualified_upper_xyz_limit_mm=.5, branch_error=branch_error,
        historical_goal_wrist_used_for_terminal_acceptance=False,
        live_joint_branch_and_reserve_guards_unchanged=True)
    sequence.report['descent_terminal_qualification'] = detail
    sequence.save()
    if detail['failed_checks']:
        raise RuntimeError('Observed terminal differs from this descent command: ' +
            ','.join(detail['failed_checks']) + '; detailed actual/command deltas saved; no grasp')
    return end


def verify_qualified_upper_return(sequence, final, qualified_upper):
    """Accept the real empty-return plan without conflating it with a wait gate.

    The 0.08-degree same_pose rule remains unchanged for stationary drift and
    historical measurement reuse. A completed MoveL is checked against its own
    computed branch, command and qualified Cartesian destination instead.
    """
    if qualified_upper is None or sequence.plan is None:
        raise RuntimeError('Current qualified upper or actual empty-return plan missing')
    plan = sequence.plan
    for arm in (final, qualified_upper, plan['start']):
        arm_shape(arm)
    start = plan['start']
    target = plan['target_sdk_pose_m_rad']
    branch_error = None
    try:
        check_segment(final, plan)
    except (ValueError, RuntimeError) as exc:
        branch_error = repr(exc)
    target_position = math.dist(final['pose'][:3], target[:3])
    target_rotation = orientation_angle_rad(final['pose'][3:], target[3:])
    upper_position = math.dist(final['pose'][:3], qualified_upper['pose'][:3])
    upper_rotation = orientation_angle_rad(final['pose'][3:], qualified_upper['pose'][3:])
    segments = sequence.report.get('arm_segments', [])
    checks = dict(
        completed_empty_return_segment=bool(segments) and segments[-1].get('phase') == 'returning_empty',
        completed_segment_matches_actual_plan=bool(segments) and segments[-1].get('plan') == plan,
        command_preserves_fresh_start_XY=target[:2] == start['pose'][:2],
        command_preserves_fresh_start_wrist=target[3:] == start['pose'][3:],
        command_uses_qualified_upper_Z=abs(target[2] - qualified_upper['pose'][2]) <= 1e-9,
        plan_frames_match_qualified_upper=start['frame_signatures'] == qualified_upper['frame_signatures'],
        plan_full_limits_match_qualified_upper=start['limits'] == qualified_upper['limits'],
        final_frames_match_qualified_upper=final['frame_signatures'] == qualified_upper['frame_signatures'],
        final_full_limits_match_qualified_upper=final['limits'] == qualified_upper['limits'],
        observed_terminal_matches_command_xyz=target_position <= .001,
        observed_terminal_matches_command_rotation=target_rotation <= .005,
        observed_upper_cartesian_xyz=upper_position <= .0005,
        observed_upper_cartesian_rotation=upper_rotation <= .005,
        observed_live_branch_and_reserves=branch_error is None)
    joint_detail = pose_comparison(final, qualified_upper)
    detail = dict(checks=checks, failed_checks=[name for name, passed in checks.items() if not passed],
        acceptance_reference='actual_empty_return_plan_and_this_cycle_frozen_upper_Cartesian_pose',
        target_sdk_pose_m_rad=copy.deepcopy(target), endpoint_xyz_error_from_command_mm=target_position*1000,
        endpoint_rotation_from_command_rad=target_rotation, command_xyz_limit_mm=1.,
        endpoint_xyz_error_from_qualified_upper_mm=upper_position*1000,
        endpoint_rotation_from_qualified_upper_rad=upper_rotation,
        qualified_upper_xyz_limit_mm=.5, rotation_limit_rad=.005, branch_error=branch_error,
        final_vs_qualified_upper_jointwise=joint_detail,
        jointwise_comparison_is_measurement_reuse_gate_not_completed_motion_gate=True,
        stationary_drift_and_live_joint_branch_guards_unchanged=True)
    sequence.report['empty_return_terminal_qualification'] = detail
    sequence.save()
    if detail['failed_checks']:
        raise RuntimeError('Empty return differs from actual plan or qualified upper: ' +
            ','.join(detail['failed_checks']) + '; detailed actual/command deltas saved')
    _prepare_profile(sequence.report['last_preflight'], sequence.profile)
    measurement_available = (
        sequence.report.get('measured_upper_whole_hand_top_clearance_mm') == 25.3
        and sequence.report.get('measured_upper_minimum_reported_clearance_mm') == 25.2
        and sequence.report.get('hand_top_clearance_measurement_basis') == MEASUREMENT_BASIS)
    reuse = joint_detail['matched'] and measurement_available
    reason = (None if reuse else 'qualified_upper_jointwise_0.08degree_measurement_reuse_gate_not_met'
        if not joint_detail['matched'] else 'reviewed_operator_measurement_record_not_available')
    source = ('operator_25.3mm_measurement_reused_at_same_freshly_matched_upper_pose' if reuse
        else 'completed_empty_return_to_qualified_Cartesian_upper_without_reusing_old_clearance')
    sequence.report.update(hand_above_target_top_confirmed=reuse,
        whole_hand_lowest_above_battery_top_mm=25.3 if reuse else None,
        minimum_reported_whole_hand_top_clearance_mm=25.2 if reuse else None,
        clearance_source=source, hand_top_clearance_source=source,
        hand_top_clearance_is_reused_operator_measurement=reuse,
        hand_top_clearance_not_reused_reason=reason,
        hand_top_clearance_verification=(
            'historical_user_measurement_and_current_pose_scene_reconfirmation_not_new_measurement' if reuse
            else 'not_measured_this_run_historical_gap_not_reused'),
        return_height_reproduced_from_qualified_upper=True,
        return_jointwise_measurement_reuse_gate_pass=joint_detail['matched'],
        return_cycle_completion_independent_of_old_clearance_reuse=True,
        lateral_transfer_ready=False, return_endpoint_is_not_lateral_clearance=True,
        physical_stop_verified=False, torque_release_verified=False)
    sequence.save()
    return final


class HighReturnCycle(SupportedCycle):
    def __init__(self, *args, high_return_context, **kwargs):
        super().__init__(*args, **kwargs)
        if (self.taught_only or not self.allow_protected_opening
                or self.enable_arm_protection or self.resume_report is not None
                or self.completed_corridor is not None):
            raise ValueError('Measured-upper visual cycle requires isolated visual read-existing-configuration mode')
        self.high_return_context = high_return_context
        self.qualified_upper = None

    def wait_input(self, index, phase, arm, values, teaching=False):
        event = super().wait_input(index, phase, arm, values, teaching=teaching)
        if index in (3, 4, 7) and event.get(SCENE_FLAG) is not True:
            raise RuntimeError('Same unmoved shell/slot/scene and measured upper clearance not reconfirmed')
        return event

    def teach_context(self, initial, tip):
        context = self.high_return_context
        old, observed = context['worker'], context['last_observed_arm']
        accepted=context['accepted_upper']
        pose_detail=pose_comparison(initial,accepted)
        endpoint_detail=pose_comparison(initial,context['accepted_upper_endpoint'])
        checks=dict(accepted_upper_pose=pose_detail['matched'],full_limits=pose_detail['full_limits_equal'],
            completed_lift_endpoint_pose=endpoint_detail['matched'],
            completed_lift_endpoint_full_limits=endpoint_detail['full_limits_equal'],
            sdk_identity=metadata_signature(self.report['sdk_environment'])==metadata_signature(old['sdk_environment']),
            actual_DH_and_identity=model_signature(self.report['fresh_model'])==model_signature(old['fresh_model']),
            hand_configuration=self.report['configuration']==old['configuration'],
            tip_parameters=tip==old['live_tip_parameters'])
        checks.update({key:self.report['arm_safety_reads'][key]==old['arm_safety_reads'][key]
            for key in SAFETY_CONFIGURATION})
        self.report['high_return_start_qualification']=dict(
            anchor_source=context['accepted_upper_source'],accepted_upper=copy.deepcopy(accepted),
            current_vs_accepted_upper=pose_detail,checks=checks,
            current_vs_completed_lift_endpoint=endpoint_detail,
            accepted_reference_policy='intersection_of_completed_endpoint_and_successful_full_preflight_not_candidate_union',
            failed_checks=[name for name,passed in checks.items() if not passed],
            rejected_wait_sample_used_as_anchor=False,
            current_vs_rejected_wait_sample_diagnostic_only=pose_comparison(initial,observed),
            no_automatic_old_point_alignment=True,tolerances_unchanged=True)
        self.save()
        failed=self.report['high_return_start_qualification']['failed_checks']
        if failed:
            raise RuntimeError('Measured-upper start qualification failed: '+','.join(failed)+
                '; detailed actual/source deltas saved; no automatic old-point alignment')
        _prepare_profile(self.report['last_preflight'], self.profile)
        goal = copy.deepcopy(old['taught_goal'])
        height = (initial['pose'][2] - goal['pose'][2]) * 1000
        if (not 30 <= height <= 120
                or math.dist(initial['pose'][:2], goal['pose'][:2]) > .0005
                or orientation_angle_rad(initial['pose'][3:], goal['pose'][3:]) > .005):
            raise RuntimeError('Fresh measured upper no longer matches the accepted vertical goal corridor')
        self.qualified_upper = copy.deepcopy(initial)
        measured = context['confirmation']
        mapped = [initial['pose'][0] * 1000, initial['pose'][1] * 1000, goal['pose'][2] * 1000]
        self.report.update(high_return_source_accepted=True,
            high_return_source_records=context['source_records'],
            high_return_operator_confirmation=copy.deepcopy(measured),
            high_return_source_accepted_upper=copy.deepcopy(accepted),
            high_return_source_anchor_kind=context['accepted_upper_source'],
            high_return_source_last_observed_arm=copy.deepcopy(observed),
            source_wait_drift_recorded=True, source_stop_ACK_is_not_physical_stop=True,
            current_qualified_upper_reference=copy.deepcopy(initial),
            taught_goal=goal, upper=copy.deepcopy(initial),
            manual_corridor_height_mm=height, board_clearance_mm=old['board_clearance_mm'],
            historical_clearances_reused=True,
            board_clearance_source='181625_noncontact_endpoint_same_scene_pending_reconfirmation',
            board_clearance_is_not_battery_clearance=True, battery_side_clearance_mm=None,
            measured_upper_whole_hand_top_clearance_mm=measured['measured_clearance_mm'],
            measured_upper_clearance_variation_mm=measured['variation_mm'],
            measured_upper_minimum_reported_clearance_mm=measured['minimum_reported_clearance_mm'],
            hand_top_clearance_measurement_basis=MEASUREMENT_BASIS,
            hand_top_clearance_source='user_statement_after_183637_lift_pending_current_scene_confirmation',
            hand_above_target_top_confirmed=False, lateral_transfer_ready=False,
            historical_target_or_corridor_reused=True, previous_vision_reused=False,
            manual_reteaching_required=False, manual_upper_required=False,
            automatic_other_slot_transfer=False, automatic_lateral_motion=False,
            no_automatic_old_point_alignment=True, camera_extrinsics_used_for_motion=False)
        self.authority.prepare_feedback_pass = True
        self.save()
        event = self.wait_input(3, 'await_vision', initial, ('b',))
        if event.get('same_taught_stationary_shell_confirmed') is not True:
            raise RuntimeError('Fresh vision must identify the same unmoved taught shell')
        document = strict_json((self.folder / 'vision/detections.json').read_bytes())
        target = select_surface(document, event['target']['index'])
        if (target != event['target'] or target['class_name'] != 'black_cyl_battery'
                or target['world_transform_sha256'] != CALIBRATION_SHA):
            raise RuntimeError('Fresh visual target/calibration differs from the fixed-scene black-shell evidence')
        point = target['surface_world_mm']
        offset = [a - b for a, b in zip(mapped, point)]
        binding = dict(target=target, surface_to_taught_TCP_mm=offset,
            original_taught_TCP_mm=[v * 1000 for v in goal['pose'][:3]],
            mapped_TCP_mm=mapped, tool_rpy_rad=initial['pose'][3:],
            scope='fresh_visual_same_qualified_upper_same_unmoved_shell_only',
            point_is_not_grasp_center=True, automatic_other_slot_transfer=False,
            automatic_lateral_motion=False, visual_captured_this_invocation=True,
            target_XY_source='this_invocation_fresh_qualified_upper_readback',
            target_Z_source='181625_original_noncontact_taught_goal',
            same_unmoved_slot_shell_mounts_and_scene_reconfirmed=True)
        self.report.update(visual_binding=binding, visual_binding_created=True,
            vision_used=True,
            vision_binding_scope='fresh_visual_same_qualified_upper_same_unmoved_shell_only',
            high_return_same_scene_reconfirmed_by_operator=True,
            board_clearance_source='original_endpoint_same_unmoved_shell_scene_reconfirmed_by_operator')
        self.save()
        (self.folder / 'visual_binding.json').write_text(
            json.dumps(binding, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
        return goal, initial, tip, mapped

    def validate_descent_terminal(self, end, goal, upper, mapped):
        self.report['historical_taught_goal_terminal_diagnostic_only'] = pose_comparison(end, goal)
        self.save()
        return verify_bound_descent(self, end, upper, mapped, self.plan)

    def verify_return_clearance(self, final):
        _prepare_profile(self.report['last_preflight'], self.profile)
        for index in (3, 4, 7):
            rows = [row for row in self.report.get('operator_inputs', []) if row.get('index') == index]
            if len(rows) != 1 or rows[0].get(SCENE_FLAG) is not True:
                raise RuntimeError('Measured-upper clearance scene confirmation missing before return acceptance')
        return verify_qualified_upper_return(self, final, self.qualified_upper)
