"""Controller-side integration for a future physically accepted route.

This release has no accepted physical route hash. It refuses before connecting,
even if JSON execution_ready flags are edited. No operator bypass switch exists.
Old accepted hand/transport code is reused without changing protection values.
"""
import copy
import hashlib
import json
import math
import time

ACCEPTED_PHYSICAL_ROUTE_SHA = None


def require_physical_route(raw):
    if ACCEPTED_PHYSICAL_ROUTE_SHA is None:
        raise RuntimeError('No physically accepted descent route in this release; no device connection')
    if hashlib.sha256(raw).hexdigest() != ACCEPTED_PHYSICAL_ROUTE_SHA:
        raise RuntimeError('Physical route evidence differs; no device connection')
    p = json.loads(raw)
    if (p.get('execution_ready') is not True or p.get('blockers')
            or p.get('whole_scene_sweep_verified') is not True
            or p.get('prepare_transition_sweep_verified') is not True
            or p.get('controller_path_validation_authorized') is not True):
        raise RuntimeError('Physical route evidence incomplete; no device connection')
    return p


class ControllerAdapter:
    """Compose reviewed Sequence/ContinuousHand functions; no new TCP parser.

    The SDK/hand session must be created only after require_physical_route.
    SDK polling uses stable target feedback; ACK never completes a phase.
    Fault inhibits hand writes and makes one arm software-stop attempt only.
    There is no fault-time open/force-zero/reposition/retry operation.
    """
    def __init__(self, sequence, permit_prepare, clock=time.monotonic, pause=time.sleep):
        self.s = sequence
        self.permit_prepare = permit_prepare
        self.clock, self.pause = clock, pause
        self.failed = False
        self.arm_attempted = False
        self.arm_stop_attempted = False

    def _check(self):
        if self.failed:
            raise RuntimeError('Workflow fault latched; no next command')

    def fresh_preflight(self, plan):
        self._check()
        # Identity/mode/native checks reuse the reviewed local launcher helper.
        from run_arm_hand_sequence import collect_sdk_environment
        metadata = collect_sdk_environment(self.s.robot)
        from run_arm_hand_sequence import NATIVE_SHA
        if (metadata.get('metadata_complete') is not True
                or metadata.get('loaded_native', {}).get('sha256') != NATIVE_SHA
                or metadata.get('api_version') != '4.3.1'
                or metadata.get('host_machine') != 'aarch64'
                or metadata.get('controller_run_mode_raw', {}).get('data') != [1]
                or metadata.get('controller_software_info', {}).get('data', [{}])[0].get('product_version') != 'RM65-BI'):
            raise RuntimeError('SDK/controller identity or real mode changed')
        arm = self.s.full_arm()
        start = plan['route_start']
        if (any(abs(a-b) > .08 for a,b in zip(arm['joints'], start['joint_degrees']))
                or math.dist([v*1000 for v in arm['pose'][:3]], start['tool_world_mm']) > .5
                or any(abs(math.atan2(math.sin(a-b), math.cos(a-b))) > .005
                       for a,b in zip(arm['pose'][3:], start['tool_rpy_rad']))):
            raise RuntimeError('Live arm is not at accepted start; no automatic alignment')
        expected = plan['current_pose']
        if arm['frame_signatures'] != expected['frame_signatures'] or arm['limits'] != expected['limits']:
            raise RuntimeError('Frames or ordinary limits changed')
        self.s.reference = dict(self.s.reference, joints=arm['joints'][:], pose=arm['pose'][:],
                                frame_signatures=arm['frame_signatures'], limits=arm['limits'])
        self.s.protection_prepared = True  # Requires fresh raw8/true, does not set values.
        low, high = self.s.safety_reads(allow_disabled=False)
        self.drive_limits = dict(minimum=low, maximum=high)
        for row in plan['numerical_witnesses']:
            if any(min(q-a, b-q) < 3 for q,a,b in zip(row['joint_degrees'], low, high)):
                raise RuntimeError('Live drive limits differ from candidate reserve')
        self.s.guard_target, self.s.guard_pose = arm['joints'][:], arm['pose'][:]
        hand = self.s.hand_factory(self.s, True)
        try:
            hand.configuration()
            state = hand.preflight(expected=self.s.profile['poses']['open'])
            return arm, state
        finally:
            self.s.close_hand()

    def prepare(self):
        self._check()
        from run_school_battery_hand import ADAPTED_PLAN
        hand = self.s.hand_factory(self.s, True)
        try:
            hand.configuration()
            hand.preflight(expected=self.s.profile['poses']['open'])
            self.permit_prepare()
            target = hand.write_pose(ADAPTED_PLAN[0], self.s.profile['poses']['open'][:])
            return hand.preflight(expected=target)
        except BaseException:
            self.fault()
            raise
        finally:
            self.s.close_hand()

    def _wait_arm(self, command, start_joints, timeout=90):
        expected_mm, expected_rpy = command['expected_tcp_world_mm'], command['expected_rpy_rad']
        target_joints = command.get('target_joint_degrees')
        began = self.clock()
        previous, stable_since, groups = None, None, 0
        while self.clock() - began < timeout:
            self._check()
            joints, pose = self.s.arm_state()
            if any(min(q-a, b-q) < 3 for q,a,b in zip(joints,
                    self.drive_limits['minimum'], self.drive_limits['maximum'])):
                raise RuntimeError('Live drive reserve fell below3degrees')
            # For joint commands also constrain all six axes to the reviewed box.
            if target_joints is not None and any(not min(a,b)-.2 <= q <= max(a,b)+.2
                    for q,a,b in zip(joints, start_joints, target_joints)):
                raise RuntimeError('Joint feedback left the reviewed segment bounds')
            reached = (math.dist([v*1000 for v in pose[:3]], expected_mm) <= .5
                and all(abs(math.atan2(math.sin(a-b), math.cos(a-b))) <= .005
                        for a,b in zip(pose[3:], expected_rpy))
                and (target_joints is None or all(abs(a-b) <= .08 for a,b in zip(joints, target_joints))))
            stationary = previous is not None and all(abs(a-b) <= .02 for a,b in zip(joints, previous[0]))
            stationary = stationary and math.dist(pose[:3], previous[1][:3]) <= .0002
            stationary = stationary and all(abs(math.atan2(math.sin(a-b), math.cos(a-b))) <= .002
                                             for a,b in zip(pose[3:], previous[1][3:]))
            if reached and stationary:
                stable_since = self.clock() if stable_since is None else stable_since
                groups += 1
                if groups >= 3 and self.clock() - stable_since >= 1:
                    self.s.guard_target, self.s.guard_pose = joints[:], pose[:]
                    return dict(joints=joints, pose=pose, feedback_observed=True, physical_stop_verified=False)
            else:
                stable_since, groups = None, 0
            previous = joints, pose
            self.pause(.1)
        raise RuntimeError('Arm target feedback timeout; no next segment')

    def command(self, command):
        self._check()
        try:
            self.s.safety_reads(allow_disabled=False)
            start_joints, _ = self.s.arm_state()
            if command['speed_percent'] != 5 or command['trajectory_connect'] != 0 or command['radius'] != 0:
                raise RuntimeError('Command policy differs')
            self.arm_attempted = True  # Before call: exceptions may follow an accepted write.
            self.s.arm_command_attempted = True
            if command['operation'] == 'controller_Movej_candidate':
                result = self.s.call('Movej_Cmd', command['target_joint_degrees'], 5,
                                     trajectory_connect=0, r=0, block=False)
            elif command['operation'] == 'controller_Movel_candidate':
                result = self.s.call('Movel_Cmd', command['target_sdk_pose_m_rad'], 5,
                                     trajectory_connect=0, r=0, block=False)
            else:
                raise RuntimeError('Unknown command type')
            if type(result) is not int or result != 0:
                raise RuntimeError('Controller command rejected or uncertain; no retry')
            return self._wait_arm(command, start_joints)
        except BaseException:
            self.fault()
            raise

    def fault(self):
        self.failed = True
        self.s.hand_motion_enabled = False
        errors = []
        if self.s.io is not None:
            try:
                self.s.io.disable_motion()
            except BaseException as exc:
                errors.append(dict(operation='inhibit_hand', error=repr(exc)))
        if self.arm_attempted and not self.arm_stop_attempted:
            self.arm_stop_attempted = True
            try:
                result = self.s.call('Move_Stop_Cmd', block=False)
                self.s.report['arm_stop_attempt'] = dict(result=result, physical_stop_verified=False)
            except BaseException as exc:
                errors.append(dict(operation='arm_stop_attempt', error=repr(exc)))
        row = self.s.report.setdefault('pregrasp_fault', dict(fault_latched=True, inhibit_errors=[],
            onsite_hardware_stop_required=True, automatic_release=False, automatic_return=False))
        row['inhibit_errors'].extend(errors)
        self.s.save()


def execute_accepted(raw, create_session):
    """Factory is never invoked for this release's unaccepted physical route."""
    plan = require_physical_route(raw)
    backend = create_session(plan)
    try:
        backend.fresh_preflight(plan)
        backend.prepare()
        for command in plan['command_specification']:
            backend.command(copy.deepcopy(command))
        return dict(outcome='pregrasp_feedback_observed', automatic_grip=False,
                    automatic_lift=False, physical_stop_verified=False)
    except BaseException:
        backend.fault()
        raise
