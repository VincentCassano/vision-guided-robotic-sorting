"""Explicit board-supported cycle opening, reusing the accepted four-word release.

An existing current-protection indication never authorizes additional closure.
A separate, manually confirmed opening capability does not clear the old latch.
"""
import copy
from vision_cycle_sequence import VisualCycle
from vision_cycle_hand import PREPARE,CAP
from run_school_battery_hand import ADAPTED_PLAN,ProtectionFault,TARGET_TOL
from high_descent_prepare_worker import same_pose


class SupportedCycle(VisualCycle):
    def __init__(self,*args,allow_protected_opening=False,**kwargs):
        super().__init__(*args,**kwargs)
        self.allow_protected_opening=allow_protected_opening
        self.release_authority=None
        self.report['explicit_supported_opening_requested']=allow_protected_opening

    def close_hand_candidate(self,hand):
        try:
            return super().close_hand_candidate(hand)
        except ProtectionFault as exc:
            if not self.allow_protected_opening:
                raise
            row=copy.deepcopy(self.report.get('last_snapshot',{}))
            prefix='Current protection or unexpected force-control status: '
            first_key=('close',0,(1135,tuple(ADAPTED_PLAN[1]['values'])))
            # Only a completely read, identity/current/arm-checked status3
            # group following the acknowledged first recipe can reach the
            # human supported-opening gate. Communication/limit faults retain
            # their normal abort, and the closed-phase latch stays terminal.
            if (str(exc)!=prefix+str(row.get('status'))
                    or first_key not in self.authority._acknowledged_writes
                    or row.get('identity')!=self.profile['device_identity']
                    or len(row.get('status',[]))!=6 or 3 not in row['status']
                    or any(v==4 for v in row['status'])
                    or any(row['status'][ch]==3 for ch in (4,5))
                    or len(row.get('targets',[]))!=6 or len(row.get('positions',[]))!=6
                    or len(row.get('current',[]))!=6
                    or any(c>lim for c,lim in zip(row['current'],self.profile['current_limit_ma']))
                    or not any(all(abs(a-b)<=TARGET_TOL for a,b in zip(row['targets'],target))
                        for target in (ADAPTED_PLAN[1]['values'],CAP))
                    or self.report['configuration']!=self.profile['observed_configuration']):
                raise
            if (self.report.get('phase')!='grasping' or self.io is None or self.io.broken
                    or (self.folder/'abort.json').exists()):
                raise
            self.report['grasp_current_protection_retained']=dict(
                error=repr(exc),feedback=row,additional_closure_disabled=True,
                original_close_capability_stays_latched=self.authority._terminal_latched,
                automatic_release=False,physical_stop_verified=False,
                next_action='await onsite r/n only after fixed board support and opening sweep confirmation')
            self.event('grasp_additional_closure_blocked_status3',**self.report['grasp_current_protection_retained'])
            self.save()
            return row['targets'][:],row

    def current_loaded_hand(self,arm,expected):
        if not self.allow_protected_opening:
            return super().current_loaded_hand(arm,expected)
        from return_after_release_worker import metadata_signature,model_signature,SAFETY_CONFIGURATION
        from sdk_environment import collect_sdk_environment
        from taught_model_readonly import collect_model_parameters
        from supported_contact_transport import PhaseAuthority
        from position_grasp_release_worker import loaded_bounds
        events=[row for row in self.report.get('operator_inputs',[]) if row.get('index')==6]
        if (len(events)!=1 or events[0].get('value') not in ('r','n')
                or events[0].get('shell_fully_supported_and_prepare_release_clear') is not True
                or self.release_authority is not None):
            raise RuntimeError('Single explicit supported-opening confirmation required')
        # A new opening-only capability is issued AFTER the r/n gate. The old
        # close token is not reset or re-enabled, even when status3 latched it.
        before_metadata=self.report['sdk_environment']
        metadata=collect_sdk_environment(self.robot)
        if metadata_signature(metadata)!=metadata_signature(before_metadata):
            raise RuntimeError('Supported opening actual SDK/real-mode identity changed')
        self.report['release_sdk_environment']=metadata;self.save()
        model=collect_model_parameters(self.folder/'fresh_model_before_supported_opening')
        self.report['fresh_model_before_supported_opening']=model;self.save()
        if model_signature(model)!=model_signature(self.report['fresh_model']):
            raise RuntimeError('Supported opening actual DH/identity changed')
        fresh=self.full_arm()
        if not same_pose(fresh,arm) or fresh['limits']!=arm['limits']:
            raise RuntimeError('Arm/frames/dynamics changed before supported opening')
        previous_safety=copy.deepcopy(self.report['arm_safety_reads'])
        low,high=self.safety_reads()
        if any(self.report['arm_safety_reads'][name]!=previous_safety[name] for name in SAFETY_CONFIGURATION):
            raise RuntimeError('Supported opening actual protections/install/drive configuration changed')
        if any(min(q-a,b-q)<3 for q,a,b in zip(fresh['joints'],low,high)):
            raise RuntimeError('Supported opening drive reserve below3degrees')
        tip=self.tip_reader()
        if tip!=self.report['live_tip_parameters']:
            raise RuntimeError('Supported opening actual tip parameters changed')
        self.guard_target,self.guard_pose=fresh['joints'][:],fresh['pose'][:]
        self.release_authority=PhaseAuthority()
        self.release_authority.allow_protected_opening=True
        self.report['supported_opening_capability']=dict(
            granted_after_permission_index=6,independent_of_close_token=True,
            original_close_latch_preserved=self.authority._terminal_latched,
            allowed_address=1135,allowed_word_count=4,allowed_values=[8000]*4,
            no_reset_or_protection_settings=True)
        self.save()
        hand=self.hand_factory(self,True)
        try:
            hand.configuration()
            critical=hand._critical_configuration()
            first=dict(hand.snapshot(),**critical);loaded_bounds(hand,first,expected)
            second=dict(hand.snapshot(),**critical);loaded_bounds(hand,second,expected)
            if any(abs(a-b)>150 for a,b in zip(first['positions'],second['positions'])):
                raise RuntimeError('Supported opening two fresh groups not observed quiet')
            self.report['supported_opening_preflight']=dict(snapshots=[first,second],
                physical_stop_verified=False,existing_status3_not_cleared=True)
            self.save()
            return hand
        except BaseException:
            self.close_hand()
            raise

    def release_hand_candidate(self,hand,targets):
        if not self.allow_protected_opening:
            return super().release_hand_candidate(hand,targets)
        if self.release_authority is None:
            raise RuntimeError('Supported opening capability missing')
        self.io.set_phase('release_loaded_position',self.release_authority)
        self.io.enable_motion();self.hand_motion_enabled=True
        row=hand.release_prepare(targets)
        self.release_authority._completed=True
        return row
