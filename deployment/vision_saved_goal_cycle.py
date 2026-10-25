"""Resume a zero-write taught goal with one authorized100mm empty lift."""
import copy
import hashlib
import math
from high_descent_prepare_worker import same_pose
from vision_cycle_hand import PREPARE
from vision_cycle_supported_release import SupportedCycle
from return_after_release_worker import (
    strict_json,arm_shape,metadata_signature,model_signature,prepare_readback,SAFETY_CONFIGURATION)

SOURCE_ERROR="RuntimeError('Upper must be World-Z30..120mm above goal, same XY/wrist; use Z jog, not freehand dragging')"


def read_saved_goal_context(folder,profile):
    current_parent=strict_json((folder/'parent_result.json').read_bytes())
    records=current_parent['saved_goal_source']
    documents={}
    for role in ('worker','parent'):
        record=records[role+'_report']
        raw=(folder/'saved_goal_source'/(role+'_result.json')).read_bytes()
        if len(raw)!=record['byte_count'] or hashlib.sha256(raw).hexdigest()!=record['sha256']:
            raise ValueError('Captured taught-goal source changed: '+role)
        documents[role]=strict_json(raw)
    old=documents['worker'];parent=documents['parent']
    if (type(old.get('schema_version')) is not int or old['schema_version']!=1
            or old.get('scope')!='fresh_taught_shell_descent_supported_grasp_release_empty_return'
            or old.get('outcome')!='aborted_no_further_writes'
            or parent.get('outcome')!=old['outcome'] or parent.get('worker_result')!=old
            or type(parent.get('worker_return_code')) is not int or parent['worker_return_code']!=1
            or parent.get('exceptions')!=[] or parent.get('worker_cleanup_timeout') is not False
            or old.get('cleanup_errors')!=[] or old.get('arm_stop_attempt') is not None
            or old.get('arm_command_attempted',False) is not False):
        raise ValueError('Only a clean zero-write manual-upper refusal can resume a taught goal')
    for key in ('arm_actuator_writes','hand_register_writes','controller_configuration_writes','hand_bus_configuration_writes'):
        if type(old.get(key)) is not int or old[key]!=0:
            raise ValueError('Source actuator/configuration write detected: '+key)
    if (len(old.get('exceptions',[]))!=1 or old['exceptions'][0].get('error')!=SOURCE_ERROR
            or old.get('vision_used') is not False or old.get('upper') is not None
            or old.get('arm_segments',[])!=[] or old.get('moves',[])!=[]):
        raise ValueError('Source is not the accepted unexecuted taught-goal stage')
    goal=old['taught_goal'];arm_shape(goal)
    if old['configuration']!=profile['observed_configuration']:
        raise ValueError('Source hand configuration differs from measured profile')
    prepare_readback(old['last_preflight'])
    if old['last_preflight']['identity']!=profile['device_identity']:
        raise ValueError('Source hand identity changed')
    metadata_signature(old['sdk_environment']);model_signature(old['fresh_model'])
    gap=old.get('board_clearance_mm')
    events=old.get('operator_inputs',[])
    goals=[row for row in events if row.get('index')==1]
    if (type(gap) not in (int,float) or not math.isfinite(gap) or gap<10
            or len(goals)!=1 or goals[0].get('noncontact_same_supported_shell') is not True
            or goals[0].get('lowest_hand_to_board_mm')!=gap):
        raise ValueError('Source taught goal lacks actual board-gap/noncontact confirmation')
    return dict(worker=old,parent=parent,source_records=records)


class SavedGoalCycle(SupportedCycle):
    def __init__(self,*args,saved_goal_context,**kwargs):
        super().__init__(*args,**kwargs)
        if not self.taught_only:raise ValueError('Saved taught goal requires explicit taught-only mode')
        self.saved_goal_context=saved_goal_context

    def teach_context(self,initial,tip):
        context=self.saved_goal_context;old=context['worker'];original=old['taught_goal']
        if (not same_pose(initial,original) or initial['limits']!=original['limits']
                or metadata_signature(self.report['sdk_environment'])!=metadata_signature(old['sdk_environment'])
                or model_signature(self.report['fresh_model'])!=model_signature(old['fresh_model'])
                or self.report['configuration']!=old['configuration'] or tip!=old['live_tip_parameters']):
            raise RuntimeError('Current saved-goal pose/device changed; no automatic old-point alignment')
        if any(self.report['arm_safety_reads'][name]!=old['arm_safety_reads'][name]
                for name in SAFETY_CONFIGURATION):
            raise RuntimeError('Current safety/installation/drive differs from taught source')
        prepare_readback(self.report['last_preflight'])
        self.authority.prepare_feedback_pass=True
        self.report.update(saved_goal_source_accepted=True,saved_goal_source_records=context['source_records'],
            saved_goal_reference=copy.deepcopy(original),taught_goal=copy.deepcopy(initial),
            board_clearance_mm=old['board_clearance_mm'],historical_clearances_reused=True,
            board_clearance_source='saved_same_pose_measurement_pending_new_u_confirmation',
            board_clearance_is_not_battery_clearance=True,battery_side_clearance_mm=None,
            actual_goal_source='this_invocation_fresh_feedback_at_the_saved_noncontact_goal',
            manual_reteaching_required=False,manual_upper_required=False,
            upward_displacement_mm=100,vision_used=False,camera_extrinsics_used_for_motion=False)
        self.save()
        event=self.wait_input(9,'await_saved_goal_lift',initial,('u',))
        if event.get('saved_goal_empty_noncontact_and_up100_sweep_clear') is not True:
            raise RuntimeError('Fresh noncontact goal, unchanged support and full100mm upward sweep not confirmed')
        self.report['board_clearance_source']='saved_same_pose_measurement_scene_reconfirmed_by_operator'
        self.report['same_goal_scene_reconfirmed_by_operator']=True
        self.save()
        self.plan=None
        upper=self.execute_vertical(initial,tip,initial['pose'][2]+.100,'lifting_saved_goal_100mm')
        self.report.update(upper=upper,automatic_corridor_height_mm=(upper['pose'][2]-initial['pose'][2])*1000,
            automatic_lift_feedback_observed=True,vision_used=False,
            target_source='this_invocation_fresh_TCP_at_saved_taught_goal',
            visual_binding_created=False,historical_target_or_corridor_reused=True)
        self.save()
        return initial,upper,tip,[value*1000 for value in initial['pose'][:3]]

    def verify_return_clearance(self,final):
        from vision_clearance_return_cycle import confirm_hand_top_clearance
        # The parent records the actual gap after this SDK worker has exited.
        confirm_hand_top_clearance(self,final)
        return self.report['final_arm']
