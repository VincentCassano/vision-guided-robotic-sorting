"""Shared guarded motion for a supervised, bounded actual visual displacement."""
import copy
import math
from high_descent_prepare_worker import same_pose
from run_arm_hand_sequence import Sequence
from vision_cycle_supported_release import SupportedCycle
from battery_grasp.high_descent_trial import PREPARE, reserve
from battery_grasp.pregrasp_replay import check_segment
from battery_grasp.real_grasp import number, orientation_angle_rad
from battery_grasp.vision_follow_motion import build_follow_segment, check_follow_segment


class FollowSequence(SupportedCycle):
    def _check_plan(self, arm):
        if self.plan.get('guard_kind') == 'follow_line':
            metrics = check_follow_segment(arm, self.plan)
            self.report['last_follow_corridor_check'] = metrics
            return metrics
        check_segment(arm, self.plan)
        return None

    def arm_state(self):
        if (self.folder/'abort.json').exists():
            raise KeyboardInterrupt('Parent abort')
        # The base reader checks arm errors and ordinary reserve, without the
        # legacy high-descent-only Z bound. This class chooses the right guard.
        joints, pose = Sequence.arm_state(self)
        if self.drive_limits is not None and reserve(joints, self.drive_limits) < 3:
            raise RuntimeError('Live drive reserve below3degrees')
        if self.plan is not None:
            self.report['last_wrist_orientation_check'] = dict(
                metric='RzRyRx_relative_rotation_angle',
                physical_rotation_rad=orientation_angle_rad(pose[3:], self.plan['start']['pose'][3:]),
                limit_rad=.005, current_rpy_rad=pose[3:],
                start_rpy_rad=self.plan['start']['pose'][3:])
            self._check_plan(dict(self.plan['start'], joints=joints, pose=pose))
        return joints, pose

    def execute_follow(self, target_pose, phase, arm=None, tip=None, vertical_limit_mm=120):
        """After the caller's onsite gate, calculate and issue exactly one MoveL.

        The caller provides scene/empty-hand permission before entry. No hand,
        protection, speed-register, frame or retry commands are issued here.
        """
        if type(vertical_limit_mm) is not int or vertical_limit_mm not in (120, 160):
            raise ValueError('Vertical limit must explicitly be120 or160mm')
        if arm is None:
            if self.guard_target is None or self.guard_pose is None or self.reference is None:
                raise RuntimeError('A freshly bound current arm is required before follow movement')
            arm = dict(self.reference, joints=self.guard_target[:], pose=self.guard_pose[:])
        if tip is None:
            tip = self.report.get('live_tip_parameters')
        if not isinstance(tip, dict):
            raise RuntimeError('Fresh actual tip parameters required before follow movement')
        if not isinstance(target_pose, (list, tuple)) or len(target_pose) != 6:
            raise ValueError('Follow request must contain six finite pose values')
        requested = [number(value, 'follow request pose') for value in target_pose]
        # The caller expresses a horizontal intent at its accepted start Z.
        # Another fresh read can differ by sub-millimetre feedback rounding.
        # Bind that intent to the fresh Z; do not interpret it as a diagonal
        # command or relax the actual straight-line monitoring thresholds.
        horizontal_intent = abs(requested[2]-arm['pose'][2]) <= 1e-9
        vertical_intent = (not horizontal_intent
                           and math.dist(requested[:2], arm['pose'][:2]) <= 1e-9)
        self.report['phase'] = 'preflight_'+phase
        self.save()
        fresh, latest = self.preflight(arm, PREPARE)
        if latest != tip:
            raise RuntimeError('Actual tip parameters changed before follow movement')
        bound_target = requested[:3]+fresh['pose'][3:]
        if horizontal_intent:
            bound_target[2] = fresh['pose'][2]
        elif vertical_intent:
            # Keep the requested absolute terminal Z, but bind the constant
            # X/Y to this full fresh preflight rather than older feedback.
            bound_target[:2] = fresh['pose'][:2]
        self.report['follow_target_binding'] = dict(
            phase=phase, requested_target_pose_m_rad=requested,
            caller_start_pose_m_rad=arm['pose'][:], fresh_start_pose_m_rad=fresh['pose'][:],
            bound_target_pose_m_rad=bound_target[:], horizontal_intent=horizontal_intent,
            vertical_intent=vertical_intent, vertical_limit_mm=vertical_limit_mm,
            requested_Z_m=requested[2], bound_Z_m=bound_target[2],
            requested_XY_m=requested[:2], bound_XY_m=bound_target[:2],
            absolute_visual_XY_preserved=not vertical_intent,
            vertical_XY_from_fresh_readback=vertical_intent, wrist_from_fresh_readback=True,
            actual_corridor_and_rotation_limits_unchanged=True)
        self.save()

        def record(row):
            if (self.folder/'abort.json').exists():
                raise KeyboardInterrupt('Cancelled during runtime calculation')
            entry = dict(row)
            event = entry.pop('event')
            self.event(event, **entry)
            self.report['last_runtime_ik_candidate'] = row
            if row.get('decision') != 'accepted' or row['attempt'] % 8 == 0:
                self.save()

        self.plan = build_follow_segment(fresh, self.drive_limits, bound_target,
            latest['line_speed_m_s'], latest['line_acc_m_s2'], record=record,
            vertical_limit_mm=vertical_limit_mm)
        if (self.folder/'abort.json').exists():
            raise KeyboardInterrupt('Cancelled before follow MoveL')
        after = self.full_arm()
        self.report['arm_after_runtime_calculation'] = after
        self.save()
        if not same_pose(after, fresh) or after['limits'] != fresh['limits']:
            raise RuntimeError('Arm/frames/complete dynamics changed during calculation; no MoveL')
        self._check_plan(after)
        self.report.update(plan=self.plan, phase=phase, execution_ready=True,
                           arm_command_attempted=True)
        self.report['arm_actuator_writes'] += 1
        self.arm_command_attempted = True
        self.save()
        result = self.call('Movel_Cmd', self.plan['target_sdk_pose_m_rad'],
            self.plan['speed_percent'], trajectory_connect=0, r=0, block=False)
        if type(result) is not int or result != 0:
            raise RuntimeError('Follow MoveL rejected/uncertain; no retry')
        endpoint = self.wait_target()
        self.report.setdefault('arm_segments', []).append(dict(
            phase=phase, plan=copy.deepcopy(self.plan), endpoint=endpoint,
            actual_world_delta_mm=[(b-a)*1000 for a, b in zip(fresh['pose'][:3], endpoint['pose'][:3])]))
        self.save()
        return endpoint

    def wait_target(self):
        began = self.clock()
        previous = None
        quiet_since = None
        groups = 0
        while self.clock()-began < self.plan['arm_timeout_seconds']:
            joints, pose = self.arm_state()
            reached = (math.dist([value*1000 for value in pose[:3]], self.plan['target_world_mm']) <= .5
                       and orientation_angle_rad(pose[3:], self.plan['start']['pose'][3:]) <= .005)
            quiet = (previous is not None
                and max(abs(a-b) for a, b in zip(joints, previous[0])) <= .02
                and math.dist(pose[:3], previous[1][:3]) <= .0002
                and orientation_angle_rad(pose[3:], previous[1][3:]) <= .002)
            if reached and quiet:
                if quiet_since is None:
                    quiet_since = self.clock()
                groups += 1
                if groups >= 3 and self.clock()-quiet_since >= 1:
                    endpoint = self.full_arm()
                    self._check_plan(endpoint)
                    if math.dist([value*1000 for value in endpoint['pose'][:3]], self.plan['target_world_mm']) > .5:
                        raise RuntimeError('Fresh endpoint differs from settled follow feedback')
                    self.guard_target, self.guard_pose = endpoint['joints'][:], endpoint['pose'][:]
                    return endpoint
            else:
                quiet_since = None
                groups = 0
            previous = joints, pose
            self.pause(.1)
        raise TimeoutError('Follow/vertical MoveL feedback deadline; no retry, open or return')
