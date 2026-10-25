"""Compile the existing field evidence into one descent workflow.

This is a command specification and feedback state machine, not an SDK adapter.
No mutable JSON flag can turn an offline candidate into device write authority.
"""
import copy
import hashlib
import json
import math
from .vertical_corridor import accepted_native_result, rotation_difference_degrees

PREPARE = [8000, 8000, 8000, 8000, 65535, 65535]
OPEN = [0, 0, 0, 0, 0, 728]
REVIEW_SHA = '12c12961f76952d9821074528f14aac16daa89ed21f3269f2e7747069dd54b28'


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def compile_workflow(native_raw, review_raw, route='taught_joint'):
    if route not in ('taught_joint', 'fixed_wrist_vertical'):
        raise ValueError('Unknown route')
    native = accepted_native_result(native_raw)
    if sha(review_raw) != REVIEW_SHA:
        raise ValueError('Measurement review provenance differs from audited162445')
    review = json.loads(review_raw)
    if (review.get('audit_pass') is not True or review.get('execution_ready') is not False
            or review.get('hardware_writes') != 0
            or review.get('correction', {}).get('corrected_mm') != 85
            or review.get('analysis', {}).get('current_hand_shape') != 'open'):
        raise ValueError('Require reviewed162445 OPEN capture and85mm operator correction')
    candidate = native['candidate']
    points = candidate['waypoints']
    current = review['analysis']['current_arm']
    joint = native['native_assessment']['joint_path']
    vertical = native['native_assessment']['vertical_path']
    if route == 'taught_joint':
        start = points[0]
        rows = joint
        commands = [dict(phase='descent_' + str(i), operation='controller_Movej_candidate',
            target_joint_degrees=point['joint_degrees'], speed_percent=5,
            trajectory_connect=0, radius=0, block=False,
            expected_tcp_world_mm=point['tool_world_mm'], expected_rpy_rad=point['tool_rpy_rad'],
            completion='fresh_feedback_stable_at_target_not_ACK',
            actual_controller_interpolation_verified=False)
            for i, point in enumerate(points[1:], 1)]
        curve = 'Two taught-endpoint joint segments;84 FK samples are retained for analysis, not84 stop-and-go commands'
    else:
        first, last = vertical[0], vertical[-1]
        start = dict(tool_world_mm=first['desired_world_mm'], tool_rpy_rad=first['desired_rpy_rad'],
                     joint_degrees=first['joint_degrees'], role='untaught_fixed_wrist_upper')
        rows = vertical
        commands = [dict(phase='vertical_descent', operation='controller_Movel_candidate',
            target_sdk_pose_m_rad=[x/1000 for x in last['desired_world_mm']] + last['desired_rpy_rad'],
            expected_tcp_world_mm=last['desired_world_mm'], expected_rpy_rad=last['desired_rpy_rad'],
            speed_percent=5, trajectory_connect=0, radius=0, block=False,
            completion='fresh_feedback_stable_at_target_not_ACK',
            actual_controller_interpolation_verified=False)]
        curve = 'One fixed-wrist World-Z candidate;44 IK/FK witnesses are not44 stop-and-go commands'
    transfer = dict(tcp_distance_mm=math.dist(current['tcp_position_mm'], start['tool_world_mm']),
        tcp_delta_world_mm=[b-a for a,b in zip(current['tcp_position_mm'], start['tool_world_mm'])],
        wrist_rotation_difference_degrees=rotation_difference_degrees(current['tcp_rpy_rad'], start['tool_rpy_rad']),
        maximum_joint_difference_degrees=max(abs(a-b) for a,b in zip(current['joint_degrees'], start['joint_degrees'])),
        current_point_is_route_start=False, transfer_path_verified=False,
        automatic_reposition_allowed=False)
    phases = [dict(phase='source_integrity', operation='verify_frozen_sources'),
        dict(phase='fresh_preflight', operation='read_identity_mode_frames_limits_protection_hand_configuration'),
        dict(phase='start_binding', operation='require_current_at_reviewed_route_start_no_auto_alignment'),
        dict(phase='scene_and_prepare_sweep', operation='require_actual_full_sweep_evidence_not_scalar_lengths'),
        dict(phase='prepare', operation='verified_empty_hand_PREPARE_only', target=PREPARE[:],
            completion='fresh_preflight_positions_targets_and_arm_stationary',
            configured_speeds_preserved=True, grasp_force_writes=False)] + commands + [
        dict(phase='terminal', operation='hold_PREPARE_at_noncontact_pregrasp',
             automatic_grip=False, automatic_lift=False, automatic_open=False, automatic_return=False)]
    missing = [dict(code='start_transfer', reason='当前点不是候选起点，未验证转移路径，不能自动回旧点。'),
        dict(code='prepare_geometry', reason='本次量尺绑定OPEN；PREPARE入口、小拇指及OPEN→PREPARE扫掠未绑定实际模型。'),
        dict(code='installed_scene_geometry', reason='安装坐标、臂/相机/线缆包络及板/电池/其他障碍World边界未齐。'),
        dict(code='controller_validation', reason='候选曲线不等于实际控制器轨迹，动态限位/跟踪/停止条件未验收。')]
    if route == 'fixed_wrist_vertical':
        missing.insert(1, dict(code='untaught_vertical_start', reason='固定腕姿高位只是逆解候选，未实际示教或验收。'))
    minimum = min(min(row['ordinary_minimum_reserve_degrees'], row['drive_minimum_reserve_degrees']) for row in rows)
    if minimum < 3:
        raise ValueError('Candidate violates required3-degree reserve')
    return dict(schema_version=1, scope='integrated_pregrasp_workflow_development',
        route=route, source_sha256=dict(native=sha(native_raw), measurement_review=sha(review_raw)),
        correction_camera_mm=85, current_pose=current, current_hand_target=OPEN[:],
        required_prepare_target=PREPARE[:], route_start=start, current_to_start=transfer,
        endpoint=points[-1], phases=phases, command_specification=commands, numerical_witnesses=rows,
        curve_scope=curve, candidate_minimum_joint_reserve_degrees=minimum,
        hardware_command_count_if_eventually_validated=len(commands),
        repeated_measurement_requested=False, candidate_generation_repeated=False,
        terminal14mm_scope='historical_PREPARE_lowest_hand_outer_surface_to_acrylic_top_only',
        actual_scene_minimum_clearance_mm=None, existing_battery_hold_acceptance_preserved=True,
        controller_adapter_source_implemented=True, controller_adapter_live_verified=False,
        hardware_session_factory_source_implemented=True, device_execution_available=False,
        command_specification_complete=True, feedback_state_machine_implemented=True,
        native_imported=False, network_connection_requested=False, hardware_writes=0,
        execution_ready=False, blockers=missing)


class FeedbackWorkflow:
    """Offline state reducer for future integration; it cannot issue any command.

    Each transition requires feedback for a single in-flight operation. A fault
    is irreversible, ignores stale success, emits stop-required and never emits
    an automatic release/return. Acknowledgment cannot complete movement.
    """
    def __init__(self, plan):
        self.plan = copy.deepcopy(plan)
        self.state = 'blocked'
        self.index = -1
        self.inflight = None
        self.events = []

    def reduce(self, event):
        event = copy.deepcopy(event)
        kind = event.get('kind')
        self.events.append(event)
        if self.state == 'fault':
            return dict(state='fault', future_writes_allowed=False)
        if kind in ('contact', 'timeout', 'communication_error', 'feedback_error', 'quit'):
            self.state = 'fault'
            return dict(state='fault', onsite_hardware_stop_required=kind != 'quit',
                software_stop_is_verified=False, automatic_release=False, automatic_return=False,
                future_writes_allowed=False)
        if kind == 'offline_review_start':
            if self.state != 'blocked':
                raise ValueError('No workflow restart')
            self.state = 'review_preflight'
            return dict(state=self.state, hardware_writes=0)
        if kind == 'preflight_reviewed':
            if self.state != 'review_preflight':
                raise ValueError('Unexpected preflight event')
            self.state = 'review_prepare'
            self.inflight = 'prepare'
        elif kind == 'ack':
            if self.inflight != event.get('phase'):
                raise ValueError('ACK phase differs')
            # This event deliberately does not advance state.
        elif kind == 'feedback_reached':
            if self.inflight != event.get('phase') or event.get('stationary') is not True:
                raise ValueError('Fresh stationary feedback for in-flight phase required')
            if self.state == 'review_prepare':
                self.index = 0
                self.state = 'review_descent'
                self.inflight = self.plan['command_specification'][0]['phase']
            elif self.state == 'review_descent':
                self.index += 1
                if self.index == len(self.plan['command_specification']):
                    self.state = 'review_terminal'
                    self.inflight = None
                else:
                    self.inflight = self.plan['command_specification'][self.index]['phase']
            else:
                raise ValueError('No outstanding motion')
        else:
            raise ValueError('Unknown offline event')
        return dict(state=self.state, inflight_phase=self.inflight, hardware_writes=0,
            device_execution_available=False, execution_ready=False)
