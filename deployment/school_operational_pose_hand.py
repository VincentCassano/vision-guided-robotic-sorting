"""Supported position-mode grasp candidate. Quiet feedback is not holding proof."""
import math
import copy
from continuous_hand_cycle import CONFIG_LENGTHS
from school_device_identity import identity_fields, same_live_identity, configuration_matches
from battery_grasp.real_grasp import orientation_angle_rad
from continuous_hand_cycle import ContinuousHand
from run_school_battery_hand import ADAPTED_PLAN,TOL,TARGET_TOL

PREPARE=[8000,8000,8000,8000,65535,65222]
CAP=[15000,40000,35000,40000,65535,65222]


class CycleHand(ContinuousHand):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self._closure_blocked_channels=set()
        self.report['feedback_policy'].update(empty_hand_only=False,
            supported_position_contact_only=True,holding_verified=False,
            no_force_target_writes=True,status5_blocks_additional_closure=True,
            protection_feedback_saved_before_application_decision=True)

    def _identity_checked(self, identity):
        identity_fields(identity)
        proof = self.report.get('startup_readiness', {})
        anchor = getattr(self.io._phase_authority, 'accepted_identity_raw', None)
        # Each raw GET was already checked against the live monotonic
        # diagnostic baseline by the transport. Configuration retains its
        # original first raw GET even if the diagnostic cleared later.
        if not proof.get('passed') or anchor is None:
            raise RuntimeError('Current identity/diagnostic lacks this-run operational evidence')
        self.report['last_identity_raw'] = identity[:]
        self.report['static_device_identity'] = identity[:6]
        self.report['last_subexception_raw'] = identity[6]

    def configuration(self):
        # Do not call inherited Hand.configuration: it conflates diagnostic
        # 1006 with immutable identity. Read the SAME full blocks explicitly.
        self._configuration_checked = False
        try:
            blocks=(('identity',1000,8),('calibration_end_absolute',1020,6),
                ('calibration_start_absolute',1030,6),('thumb_presets_absolute',1040,3),
                ('pid_p',1045,6),('pid_i',1055,6),('pid_d',1065,6),('pid_g',1075,6),
                ('current_limit_ma',1095,6),('stall_speed',1185,6),('stall_current_ma',1195,6),
                ('stall_stop_ms',1205,6),('stall_retry_ms',1215,6))
            self.report['configuration']={}
            for name,address,count in blocks:
                self.report['configuration'][name]=self.read(name,address,count)
                self.save()
            configuration=self.report['configuration']
            for name,length in CONFIG_LENGTHS.items():
                self._u16(configuration[name],length,name)
            self._identity_checked(configuration['identity'])
            if not configuration_matches(configuration,self.sequence.profile['observed_configuration']):
                raise RuntimeError('Full calibration/PID/protection/static identity differs')
            self._configuration_checked=True
            self.report.setdefault('configuration_checks',[]).append(dict(
                monotonic=self.clock(), values=copy.deepcopy(configuration),
                phase='entry', matched=True, static_identity_and_all_settings_matched=True,
                diagnostic_raw=configuration['identity'][6], diagnostic_not_a_setting=True))
            self.save()
        except BaseException:
            self._disable()
            raise

    def guard(self):
        try:
            joints,pose=self.sequence.arm_state()
            angle=orientation_angle_rad(pose[3:],self.expected_pose[3:])
            if (any(abs(a-b)>.08 for a,b in zip(joints,self.expected_arm))
                    or math.dist(pose[:3],self.expected_pose[:3])>.0005 or angle>.005):
                self.event('arm_drift_during_hand_phase',joints=joints,pose=pose,
                    expected_joints=self.expected_arm,expected_pose=self.expected_pose,
                    physical_rotation_rad=angle,rotation_limit_rad=.005)
                raise RuntimeError('Arm drift during hand phase; no further hand writes')
        except BaseException:
            self._disable()
            raise

    def read_feedback_group(self):
        """Default wire order; specialised entries may put status last."""
        began=self.clock()
        identity=self.read('identity_feedback',1000,8)
        status=self.read('status',1085,6)
        current=self.read('current_ma_0_to_4',1105,5)
        positions=self.read('positions',1145,6)
        rotation_began=self.clock();rotation_current=self.io.single(1110)
        self.event('read',name='rotation_current_ma',address=1110,
            values=[rotation_current],started_monotonic=rotation_began)
        targets=self.read('targets_during_motion',1135,6)
        return dict(identity=identity,status=status,current=current+[rotation_current],
            positions=positions,targets=targets,reads_are_sequential=True,
            started_monotonic=began,finished_monotonic=self.clock(),
            physical_stop_verified=False,torque_release_verified=False)

    def snapshot(self):
        """Save a whole fresh group even when a status protection is present."""
        try:
            row=self.read_feedback_group()
            identity,status,positions,targets=(row[k] for k in ('identity','status','positions','targets'))
            # A protection indication must not leave last_snapshot pointing to
            # an old PREPARE group. These extra operations are reads only.
            self.report['last_snapshot']=row;self.last_feedback=row
            self.event('complete_position_contact_feedback',snapshot=row)
            self.save()
            self._identity_checked(identity)
            for values,length,name in ((status,6,'status'),(row['current'],6,'current'),
                    (positions,6,'position'),(targets,6,'target')):
                self._u16(values,length,name)
            if any(v not in range(6) for v in status):
                self._protection('Invalid status feedback: '+str(status))
            if 5 in status:
                self._closure_blocked_channels.update(ch for ch,v in enumerate(status) if v==5)
                self.report['closure_blocked_channels']=sorted(self._closure_blocked_channels)
                self.report['status5_warning_count']=self.report.get('status5_warning_count',0)+1
                self.event('status5_warning',snapshot=row,
                    message='Keep existing target; do not add closure on a status5 channel',
                    no_retry_or_target_write=True,hardware_drive_disabled=False)
                self.save()
            self.guard()
            limits=self.report['configuration']['current_limit_ma']
            if any(limit<=0 or c>limit for c,limit in zip(row['current'],limits)):
                self._protection('Measured current exceeds live controller current limit')
            self.check_status(row)
            self.save()
            return row
        except BaseException:
            self._disable();raise

    def check_status(self,row):
        # Normal grasping retains the existing protections. An isolated,
        # explicitly selected opening-only recovery may specialize this check.
        if any(v in (3,4) for v in row['status']):
            self._protection('Current protection or unexpected force-control status: '+str(row['status']))

    def check_targets(self,row,expected):
        if any(abs(a-b)>TARGET_TOL for a,b in zip(row['targets'],expected)):
            raise RuntimeError('Target changed outside this cycle')
        if any(p>cap+TOL for p,cap in zip(row['positions'],CAP)):
            raise RuntimeError('Actual finger position exceeded accepted position sweep')

    def observe_position_contact(self,expected,release=False):
        began=self.clock();low=high=None;groups=0;quiet_since=None
        while self.clock()-began<30:
            row=self.snapshot();self.check_targets(row,expected)
            p=row['positions']
            if release:
                good=all(abs(a-b)<=TOL for a,b in zip(p,expected))
            else:
                if low is None:low=p[:];high=p[:]
                low=[min(a,b) for a,b in zip(low,p)];high=[max(a,b) for a,b in zip(high,p)]
                good=max(b-a for a,b in zip(low,high))<=TOL
            if good:
                if quiet_since is None:quiet_since=self.clock()
                groups+=1
                if groups>=3 and self.clock()-quiet_since>=1:
                    # Same boundary policy as the accepted ContinuousHand:
                    # snapshot checks identity/current/targets/arm every group;
                    # configuration is checked at operation boundaries.
                    self._critical_configuration()
                    self.event('controlled_contact_observed',snapshot=row,positions_may_be_partial=not release,
                        calibrated_force_or_holding_verified=False)
                    return row
            else:low=p[:];high=p[:];quiet_since=None;groups=0
        self._disable()
        raise TimeoutError('Position-mode contact/release did not settle; no retry')

    def close_candidate(self,previous):
        expected=previous[:]
        for step in ADAPTED_PLAN[1:]:
            self._critical_configuration();before=self.snapshot();self.check_targets(before,expected);self.guard()
            first=step['address']-1135
            blocked=[ch for ch in range(first,first+step['count'])
                if step['values'][ch-first]>expected[ch] and ch in self._closure_blocked_channels]
            if blocked:
                self.report['closure_paused_status5']=dict(channels=blocked,
                    skipped_step=step,targets_kept=expected[:],holding_verified=False,
                    hardware_drive_disabled=False)
                self.event('additional_closure_skipped_status5',**self.report['closure_paused_status5'])
                self.save()
                return expected,before
            if not self.sequence.hand_motion_enabled or not self.io.motion_enabled:
                raise RuntimeError('Controlled contact not authorized')
            self.event('pose_write_before',address=step['address'],values=step['values'])
            self.io.write_group(step['address'],step['values'])
            start=step['address']-1135;expected[start:start+step['count']]=step['values']
            row=self.observe_position_contact(expected)
            self.report.setdefault('grasp_steps',[]).append(dict(step=step,feedback=row,
                partial_position_allowed_for_intended_contact=True,holding_verified=False))
            self.save()
        return expected,row

    def release_prepare(self,expected):
        self._critical_configuration();self.check_targets(self.snapshot(),expected);self.guard()
        if not self.sequence.hand_motion_enabled or not self.io.motion_enabled:raise RuntimeError('Release not authorized')
        self.io.write_group(1135,PREPARE)
        return self.observe_position_contact(PREPARE,release=True)
