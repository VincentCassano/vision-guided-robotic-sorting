#!/usr/bin/env python3
"""Onsite visual binding, taught-Z descent, supported grip/release, empty return."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
import math
from pathlib import Path
import platform
import signal
import subprocess
import sys
import time
import zipfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/battery_grasp/src'))
from run_localized_pregrasp import capture_vision,CALIBRATION_SHA

SUCCESS='completed_visual_descent_grasp_release_empty_return'
TAUGHT_SUCCESS='completed_taught_descent_grasp_release_empty_return'
FOLLOW_SUCCESS='completed_adjacent_groove_visual_follow_grasp_release_empty_return'
TERMINAL_SUCCESS='completed_visual_grasp_release_empty_return_from_saved_terminal'
FOLLOW_SCOPE='qualified_adjacent_groove_116mm_visual_follow_cycle'
CLEARANCE_SUCCESS='completed_empty_vertical_retreat_top_clearance_confirmed'
CLEARANCE_INSUFFICIENT='completed_empty_vertical_retreat_clearance_insufficient'
CLEARANCE_PENDING='completed_empty_vertical_retreat_measurement_pending'
CLEARANCE_UNMEASURED='completed_empty_vertical_retreat_clearance_unmeasured'
SUCCESS_OUTCOMES=(SUCCESS,TAUGHT_SUCCESS,FOLLOW_SUCCESS,TERMINAL_SUCCESS,CLEARANCE_SUCCESS,CLEARANCE_INSUFFICIENT,CLEARANCE_PENDING,CLEARANCE_UNMEASURED)
PHASES={0:('await_prepare',('h',)),1:('await_goal',('t',)),
    2:('await_manual_upper',('v',)),3:('await_vision',('b',)),
    4:('await_descend',('d',)),5:('await_grasp',('g',)),
    6:('await_supported_release',('r','n')),7:('await_empty_return',('u',)),
    8:('await_camera_clearance_lift',('h',)),9:('await_saved_goal_lift',('u',)),
    10:('await_clearance_lift',('u',)),11:('await_hand_top_clearance',('c',))}
FLAGS={0:'empty_hand_high_clear_confirmed',1:'noncontact_same_supported_shell',
    2:'onsite_world_Z_corridor_rehearsed_noncontact',3:'same_taught_stationary_shell_confirmed',
    4:'same_corridor_scene_and_stop_confirmed',5:'only_intended_shell_contact_and_closure_clear',
    6:'shell_fully_supported_and_prepare_release_clear',7:'shell_on_board_hand_empty_and_corridor_clear',
    8:'camera_clearance_lift_confirmed',9:'saved_goal_empty_noncontact_and_up100_sweep_clear',
    10:'empty_hand_upward_clearance_sweep_confirmed',11:'whole_hand_top_clearance_measured'}


def installed():
    hashes={}
    for line in (ROOT/'deployment/vision-grasp-cycle-v20-files.sha256').read_text(encoding='ascii').splitlines():
        digest,name=line.split('  ',1);path=(ROOT/name).resolve()
        if ROOT.resolve() not in path.parents or name in hashes or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
            raise ValueError('Installed cycle payload changed: '+name+'; no connection')
        hashes[name]=digest
    return hashes


def permission(index,phase,answer,target=None,completed_corridor=False,follow_vision=False,measured_shift_mm=None,high_return=False):
    expected,values=PHASES[index]
    if phase!=expected:raise ValueError('Wrong cycle phase')
    value=answer.strip().lower();row={}
    if index==1 and value!='q':
        try:gap=float(value)
        except ValueError:raise ValueError('输入板面垂直间隙mm，或q；不是电池与手指间隙')
        if not math.isfinite(gap) or gap<10:raise ValueError('手部最低外表面到板顶须至少10mm；不能确认就q')
        value='t';row['lowest_hand_to_board_mm']=gap
    if index==11 and value!='q':
        try:gap=float(value)
        except ValueError:raise ValueError('输入整手最低外表面高于电池顶面的垂直净空mm；未知或仍低于顶面就q')
        if not math.isfinite(gap) or gap<0:raise ValueError('请输入非负实测mm；仍低于电池顶面或不能确认就q，不得横移')
        value='c';row['whole_hand_lowest_above_battery_top_mm']=gap
    if value not in values+('q',):raise ValueError('输入 '+('/'.join(values))+' 或q')
    row.update(value=value,index=index,phase=phase,timestamp_utc=datetime.now(timezone.utc).isoformat())
    row[FLAGS[index]]=value!='q'
    if index==3 and value=='b':
        if target is None:raise ValueError('Missing fresh visual target')
        row['target']=target
    if completed_corridor and index in (3,4):
        row['same_original_shell_mounts_and_scene_unchanged']=value!='q'
    if high_return and index in (3,4,7):
        row['same_high_return_shell_scene_and_clearance_confirmed']=value!='q'
    if follow_vision and index in (3,4):
        row['adjacent_groove_same_height_and_pose_confirmed']=value!='q'
        if value!='q':
            if type(measured_shift_mm) is not int or measured_shift_mm not in (-116,116):
                raise ValueError('Missing independent onsite left/right adjacent-groove direction; no motion')
            row['measured_world_X_shift_mm']=measured_shift_mm
    return row


def write_permission(folder,row):
    target=folder/'permissions'/('%d.json'%row['index'])
    if target.exists():raise ValueError('Permission already used; no repeat')
    temp=target.with_suffix('.tmp')
    temp.write_text(json.dumps(row,ensure_ascii=False)+'\n',encoding='utf8');temp.replace(target)


def prepare_resume(source,folder,report,save):
    source=source.resolve();raw=source.read_bytes();old=json.loads(raw)
    parent=json.loads((source.parent/'parent_result.json').read_bytes())
    if parent.get('worker_result')!=old or parent.get('worker_return_code')!=1:
        raise ValueError('Resume parent/worker records differ; no connection')
    destination=folder/'resume_source';destination.mkdir()
    copied=destination/'worker_result.json';copied.write_bytes(raw)
    (destination/'parent_result.json').write_bytes((source.parent/'parent_result.json').read_bytes())
    vision=folder/'vision';vision.mkdir()
    for name in ('detections.json','original.png','detections.png','aligned_depth_u16.png'):
        (vision/name).write_bytes((source.parent/'vision'/name).read_bytes())
    if hashlib.sha256((vision/'detections.json').read_bytes()).hexdigest()!=parent['vision_source']['detections_sha256']:
        raise ValueError('Historical visual evidence changed; no connection')
    report.update(resume_requested_source=str(source),resume_source_sha256=hashlib.sha256(raw).hexdigest(),
        target=parent['target'],vision_source=dict(parent['vision_source'],captured_in_this_invocation=False,
        reuse_reason='v1_zero_motion_numerical_refusal_same_scene_reconfirmation_required'))
    save();return copied


def capture_corridor_sources(args,folder,report,save):
    destination=folder/'sources';destination.mkdir()
    sources={}
    for role in ('grasp','release'):
        path=getattr(args,'reuse_'+role+'_report').resolve();raw=path.read_bytes()
        copied=destination/(role+'_worker_result.json');copied.write_bytes(raw)
        sources[role]=dict(original_path=str(path),captured_path=str(copied.relative_to(folder)),
            byte_count=len(raw),sha256=hashlib.sha256(raw).hexdigest())
    report.update(sources=sources,completed_corridor_reuse_requested=True,
        source_role='original_completed_descent_and_supported_opening_provenance_only',
        prior_pose_not_used_as_current_state=True,previous_vision_not_reused=True,
        fresh_visual_capture_required=True,automatic_old_point_alignment=False,
        automatic_other_slot_transfer=False)
    save()


def capture_follow_source(source,folder,report,save):
    source=source.resolve()
    raw=source.read_bytes();old=json.loads(raw)
    parent_raw=(source.parent/'parent_result.json').read_bytes()
    parent=json.loads(parent_raw)
    if (source.name!='worker_result.json' or parent.get('worker_result')!=old
            or type(parent.get('worker_return_code')) is not int or parent['worker_return_code']!=0
            or old.get('outcome')!=SUCCESS or parent.get('outcome')!=SUCCESS
            or parent.get('exceptions') or old.get('exceptions') or old.get('cleanup_errors')):
        raise ValueError('Visual-follow source must be one complete successful visual cycle with exact parent/worker evidence; no connection')
    reference_raw={name:(source.parent/'vision'/name).read_bytes()
        for name in ('detections.json','original.png','detections.png','aligned_depth_u16.png')}
    if (parent.get('vision_source',{}).get('captured_in_this_invocation') is not True
            or hashlib.sha256(reference_raw['detections.json']).hexdigest()
                !=parent.get('vision_source',{}).get('detections_sha256')):
        raise ValueError('Successful baseline fresh visual evidence/hash missing or changed; no connection')
    destination=folder/'follow_source';destination.mkdir()
    copied=destination/'worker_result.json';copied.write_bytes(raw)
    (destination/'parent_result.json').write_bytes(parent_raw)
    vision=folder/'reference_vision';vision.mkdir()
    for name,data in reference_raw.items():(vision/name).write_bytes(data)
    sources={
        'worker_report':dict(original_path=str(source),captured_path=str(copied.relative_to(folder)),
            byte_count=len(raw),sha256=hashlib.sha256(raw).hexdigest()),
        'parent_report':dict(original_path=str(source.parent/'parent_result.json'),
            captured_path='follow_source/parent_result.json',byte_count=len(parent_raw),
            sha256=hashlib.sha256(parent_raw).hexdigest()),
        'reference_vision':dict(detections_sha256=hashlib.sha256(reference_raw['detections.json']).hexdigest(),
            files={name:dict(captured_path='reference_vision/'+name,
            byte_count=len(data),sha256=hashlib.sha256(data).hexdigest())
            for name,data in reference_raw.items()})}
    report.update(follow_source=sources,follow_requested_source=str(source),
        follow_source_sha256=sources['worker_report']['sha256'],previous_vision_not_reused=True,
        fresh_visual_capture_required=True,prior_pose_not_used_as_current_state=True,
        camera_global_extrinsic_accuracy_certified=False)
    save();return copied


def capture_saved_goal_source(source,folder,report,save):
    source=source.resolve()
    if source.name!='worker_result.json':raise ValueError('Select the actual taught-goal worker_result.json')
    raw=source.read_bytes();parent_raw=(source.parent/'parent_result.json').read_bytes()
    old=json.loads(raw);parent=json.loads(parent_raw)
    if parent.get('worker_result')!=old or parent.get('worker_return_code')!=1:
        raise ValueError('Saved-goal parent/worker mismatch; no connection')
    destination=folder/'saved_goal_source';destination.mkdir()
    records={}
    for role,data in (('worker',raw),('parent',parent_raw)):
        name=role+'_result.json';(destination/name).write_bytes(data)
        records[role+'_report']=dict(original_path=str(source if role=='worker' else source.parent/name),
            captured_path='saved_goal_source/'+name,byte_count=len(data),sha256=hashlib.sha256(data).hexdigest())
    report.update(saved_goal_source=records,previous_result_reused=True,
        saved_goal_automatic100_requested=True,no_automatic_old_point_alignment=True)
    save();return destination/'worker_result.json'


def capture_clearance_source(source,folder,report,save):
    source=source.resolve()
    if source.name!='worker_result.json':raise ValueError('Select the completed cycle worker_result.json')
    raw=source.read_bytes();parent_raw=(source.parent/'parent_result.json').read_bytes()
    old=json.loads(raw);parent=json.loads(parent_raw)
    if (parent.get('worker_result')!=old or type(parent.get('worker_return_code')) is not int
            or parent['worker_return_code']!=0 or old.get('outcome')!=TAUGHT_SUCCESS
            or parent.get('outcome')!=TAUGHT_SUCCESS):
        raise ValueError('Clearance continuation requires exact completed taught cycle evidence; no connection')
    destination=folder/'clearance_source';destination.mkdir()
    records={}
    for role,data in (('worker',raw),('parent',parent_raw)):
        name=role+'_result.json';(destination/name).write_bytes(data)
        records[role+'_report']=dict(original_path=str(source if role=='worker' else source.parent/name),
            captured_path='clearance_source/'+name,byte_count=len(data),sha256=hashlib.sha256(data).hexdigest())
    report.update(clearance_source=records,previous_result_reused=True,
        clearance_return_requested=True,no_automatic_old_point_alignment=True)
    save();return destination/'worker_result.json'


def operator_prompt(index,state):
    if index==10:
        print('刚才整轮已完成，本次不下降、不闭指，只续升约',round(state['clearance_requested_additional_up_mm'],3),'mm，到预抓位上方100mm；不横移。',flush=True)
        return '确认电池仍由板承托、手已完全脱离且空手PREPARE；退出示教并停稳、双手撤离，整臂/手/腕相机/线缆向上扫掠无遮挡，现场守急停。u=仅续升；q=原位退出：'
    if index==11:
        print('上升反馈完成。测量【整只手最低外表面】比【电池顶部水平面】高多少mm，包含弯曲小拇指；手在电池旁边也按同一水平面比较。',flush=True)
        return '实际停稳后量尺；不是到板面或斜向距离。输入实测mm（20mm及以上才记录高度通过；不足也可如实输入）；还低于电池顶面/未知就q。此步骤不授权横移：'
    if index==9:
        print('已复用这次新预抓位；程序升100mm再下降抓放，空返后实测整手底到电池顶净空。原板面间隙：',state.get('board_clearance_mm'),'mm。',flush=True)
        return '确认当前仍空手PREPARE、与所存预抓位相同且不碰电池/板，电池和板未动；退出示教、实际停稳、双手撤离，向上100mm整臂/手/腕相机/线缆扫掠无遮挡，现场守急停。u=程序升100mm；q=保持退出：'
    if index==8:
        if state.get('follow_start_region',{}).get('accepted_region')=='already_40mm_raised':
            print('当前已在原上方+40mm，不再上升；电池已搬好，保持当前位置，不再搬一次。',flush=True)
            return '确认空手PREPARE、退出示教且实际停稳，后续整臂、手、腕相机和线缆通道无遮挡，独占控制、现场守急停。h=确认后先重新读取复核，再采移后图；q=不动作退出：'
        print('本轮先保持PREPARE空手，仅沿World Z+上升40mm，为移后视觉采图让开遮挡；电池已搬好，保持当前位置，不再搬一次。',flush=True)
        return '确认空手PREPARE、退出示教且实际停稳，向上40mm整段的臂身、手、腕相机与线缆扫掠均无障碍，独占控制、现场守急停。h=上升40mm后开始采图；q=不动作退出：'
    if index==0:
        return '自检已结束；空手高位、完整PREPARE活动范围距外物至少150mm、臂停稳。h=执行一次PREPARE，q=退出：'
    if index==1:
        print('现在允许示教：将PREPARE空手摆到真正预抓位，电池立在板上，位于拇指和四指之间；不夹紧、不接触。',flush=True)
        print('这一步可以改变臂姿；摆好后退出示教、停稳、撤开双手。只量整手最低外表面到亚克力板顶的垂直间隙。',flush=True)
        return '确认各手指与电池仍未接触，输入板面间隙mm（至少10，可小数；不要输入手到电池距离），q=退出：'
    if index==2:
        print('已记录本次预抓TCP(mm)：',[v*1000 for v in state['taught_goal']['pose'][:3]],flush=True)
        print('现在只用示教的World Z+笛卡尔点动向上升高100mm（允许30..120mm）；X/Y与腕姿不变。不要徒手拖动。',flush=True)
        print('现场观察这一整段：整手、腕相机、臂身和线缆均不得碰到电池/板/外物。程序在本提示等待期间不查询或驱动设备。',flush=True)
        return '上升完退出示教，实际停稳、双手撤开。v=确认整段无接触并记录上方点，q=退出：'
    if index==3:
        if state.get('high_return_source_accepted'):
            print('当前已在约100mm高位，净空实测25.3±0.1mm已记录；不再升高、示教或量尺。画面须为当前同一槽位的黑色空壳，不要搬动电池。',flush=True)
            return '确认新图对应刚才抓放的同一空壳且仍在原槽，手型/板/头部/底座/升降/安装及周边物件未改，已量高位净空仍适用。b=绑定本轮视觉与已教新腕姿预抓位；q=不动作退出：'
        if state.get('scope')==FOLLOW_SCOPE:
            print('刚才按S保存的必须是【搬入相邻槽以后】的画面。若保存后才搬了空壳，输入q；重开程序只重采当前移后图，不要再搬一次。',flush=True)
            if state.get('fixture_indexed_116mm_requested'):
                print('本轮按已量116mm槽距和现场选择方向确定TCP横移，视觉核对新槽；保留原Y和预抓终点Z。不是任意视觉抓心定位。',flush=True)
                return '确认同一个空壳已在所选方向相邻完整槽完全落底站稳，槽距116mm、支撑高度与姿态相同，视觉框确实对应新槽目标；头部/底座/升降/板/安装均未改变。b=绑定本次116mm槽位横移与视觉核对；q=退出：'
            return '确认同一个空壳已放到刚才现场选择方向的相邻完整槽，完全落底站稳；槽圆心距116mm，同侧边缘位置差也为116mm，支撑高度和姿态相同；头部/底座/升降/板/安装均未改变。不要小幅推出槽边，不选前后或斜向槽。b=绑定本次视觉位移，q=退出：'
        if state.get('completed_corridor_reuse'):
            return '已复用完成过下降的固定通道；确认新视觉目标就是原先同一个空壳，同一板上位置，安装/头部/底座/升降/板未改变。b=绑定本次目标，q=退出：'
        return '确认刚选的视觉目标就是刚示教的同一个空壳，空壳/板/头部/底座/升降均未移动。b=绑定目标，q=退出：'
    if index==4:
        if state.get('high_return_source_accepted'):
            print('从本次读取的当前高位，仅沿World Z下降约100mm到已教预抓Z；X/Y和腕姿保持当前值，不横移。',flush=True)
            return '确认仍为原槽同一空壳，整臂/手/腕相机/线缆原竖直通道无障碍，场景/手型/已量高位净空未变，现场守急停。d=下降；q=留在高位：'
        if state.get('saved_goal_source_accepted'):
            return '程序已完成上升100mm；确认刚才整段通道仍无遮挡、电池/板未动、独占控制及现场急停就绪。d=自动下降回本轮确认的预抓位；q=保持上方：'
        if state.get('scope')==FOLLOW_SCOPE:
            print('本次视觉位移与目标TCP：',json.dumps(state.get('follow_binding',{}),ensure_ascii=False),flush=True)
            if state.get('fixture_indexed_116mm_requested'):
                print('本轮TCP横移采用已量槽距和已绑定现场方向（机器人左/+116mm或右/-116mm），视觉只核对新槽；保留原Y和预抓终点Z，不是任意视觉抓心定位。目标方向、槽位或数值不符就q。',flush=True)
                return '确认所选相邻完整槽完全落底站稳、槽距116mm且支撑高度/姿态不变；横移和下降通道无遮挡，独占控制及现场急停就绪。d=按已绑定116mm槽位横移并下降；q=留在上方：'
            print('核对上述视觉位移与刚才独立选择的机器人左/+116mm或右/-116mm相符；若方向/距离或目标明显错误，输入q。本阶段不能换方向。d先在上方横移，再下降、夹持/松指并空手返回。',flush=True)
            return '确认同空壳已在所选方向相邻完整槽完全落底站稳、槽距116mm、支撑高度/姿态不变，上方横移与下降通道无遮挡，独占控制及现场急停就绪。d=按已绑定方向横移并下降；q=留在上方：'
        if state.get('completed_corridor_reuse'):
            print('本次从重新读取的上方点下降，保持本次X/Y与腕姿；目标Z复用原预抓位，不自动横移或回旧点。',flush=True)
            print('确认同一个空壳仍在原板上位置，原完整下降通道无遮挡、现场守急停；位置或安装改变就q。',flush=True)
            return '确认原下降通道仍无遮挡、场景未变、独占控制及现场急停就绪。d=下降到原预抓Z；q=留在上方：'
        if state.get('resume'):
            print('已复用这次原图、板面间隙记录',state['board_clearance_mm'],'mm和示教通道；当前位置已重新读取，没有回旧点动作。',flush=True)
            print('现场必须确认头部、底座、升降、板和同一空壳均未改变；变化了就q，不套用旧通道。',flush=True)
        return '保持独占控制，确认刚才示教走过的通道仍无遮挡、现场守急停。d=自动下降回本次预抓位，q=留在上方：'
    if index==5:
        if state.get('terminal_resume_source_accepted'):
            print('上一轮下降已经完成，本次只从当前预抓位继续夹持/松指/空返；不再采图或下降。',flush=True)
            print('确认仍是已绑定原槽空壳、板/安装/周边及手型未改；臂已实际停稳，只有预定手指可接触目标空壳。',flush=True)
        print('已到预抓终点；下一步才允许手指接触目标空壳。臂保持静止，不提起电池。',flush=True)
        if state.get('explicit_supported_opening_requested'):
            print('状态5本身不抛错；若出现已核对的状态3，禁止追加闭合，转到现场确认后松指。状态4、真实超限或通信错误仍停止流程。',flush=True)
        return '确认空壳由板面承托、闭指范围不碰板或外物、双手退出。g=学校目标夹持（最多两步，保护时不再追加）；q=保持：'
    if index==6:
        if state.get('grasp_current_protection_retained'):
            print('状态3已记录，追加闭合已禁用，原闭合权限保持锁定；这不是保护解除或夹持成功证明。',flush=True)
        print('位置读数已记录；这不证明承重。现场观察是否夹住，不用手托电池，也不移动臂。',flush=True)
        if state.get('explicit_supported_opening_requested'):
            return '确认空壳全部重量由板面承托、松指路径无遮挡。r=夹住后只松拇指屈曲/食指/中指/无名指至8000，保留小指/对掌；n=未夹好但同样安全松开；q=保持：'
        return '确认空壳全部重量由板面承托、松指路径无遮挡。r=观察夹持成功并松至PREPARE；n=未夹好但同样安全松开；q=保持：'
    if index==7 and state.get('terminal_resume_source_accepted'):
        return '确认空壳仍在原槽由板承托、手已完全脱离且空手PREPARE，整臂/手/腕相机/线缆原上升通道清空，场景未改。u=仅空返原上方Z；q=留在当前位：'
    if index==7 and state.get('high_return_source_accepted'):
        return '确认空壳仍在原槽由板承托、手已完全脱离且空手PREPARE，原向上通道清空，板/物件/手型及已量25.3mm高位净空仍适用。u=空手回本轮高位；q=原位退出：'
    return '确认电池仍在板上、手已完全脱离且空手、原上升通道无接触。u=空手升回上方点，q=原位退出：'


def capture_high_return_source(source,folder,report,save):
    source=source.resolve()
    if source.name!='worker_result.json':raise ValueError('Select the actual completed-ascent worker_result.json')
    destination=folder/'high_return_source';destination.mkdir()
    records={}
    for role in ('worker','parent'):
        original=source if role=='worker' else source.parent/'parent_result.json'
        data=original.read_bytes();name=role+'_result.json';(destination/name).write_bytes(data)
        records[role+'_report']=dict(original_path=str(original),captured_path='high_return_source/'+name,
            byte_count=len(data),sha256=hashlib.sha256(data).hexdigest())
    original=ROOT/'deployment/clearance_183637_operator_confirmation.json'
    data=original.read_bytes();(destination/'confirmation.json').write_bytes(data)
    records['confirmation']=dict(original_path=str(original),captured_path='high_return_source/confirmation.json',
        byte_count=len(data),sha256=hashlib.sha256(data).hexdigest())
    report.update(high_return_source=records,previous_result_reused=True,
        actual_top_clearance_source='direct_user_reply_25.3_plus_minus0.1mm',
        previous_vision_not_reused=True,fresh_visual_capture_required=True,
        no_automatic_old_point_alignment=True,lateral_transfer_ready=False)
    save();return destination/'worker_result.json'


def capture_terminal_resume_source(source,folder,report,save):
    source=source.resolve()
    if source.name!='worker_result.json':raise ValueError('Select the actual completed-descent worker_result.json')
    destination=folder/'terminal_resume_source';destination.mkdir()
    records={}
    for role in ('worker','parent'):
        original=source if role=='worker' else source.parent/'parent_result.json'
        data=original.read_bytes();name=role+'_result.json';(destination/name).write_bytes(data)
        records[role+'_report']=dict(original_path=str(original),captured_path='terminal_resume_source/'+name,
            byte_count=len(data),sha256=hashlib.sha256(data).hexdigest())
    original_parent=json.loads((destination/'parent_result.json').read_bytes())
    vision=folder/'vision';vision.mkdir()
    records['vision_files']={}
    for name in ('detections.json','original.png','detections.png','aligned_depth_u16.png'):
        data=(source.parent/'vision'/name).read_bytes();(vision/name).write_bytes(data)
        records['vision_files'][name]=dict(captured_path='vision/'+name,byte_count=len(data),
            sha256=hashlib.sha256(data).hexdigest())
    data=(source.parent/'visual_binding.json').read_bytes();(folder/'visual_binding.json').write_bytes(data)
    records['visual_binding']=dict(captured_path='visual_binding.json',byte_count=len(data),
        sha256=hashlib.sha256(data).hexdigest())
    report.update(terminal_resume_source=records,previous_result_reused=True,
        vision_source=dict(original_parent['vision_source'],captured_in_this_invocation=False,
            reuse_reason='completed_descent_same_shell_scene_reconfirmation_before_grip'),
        target=original_parent['target'],previous_vision_reused=True,fresh_visual_capture_required=False,
        no_automatic_old_point_alignment=True,descent_already_completed_in_source=True,
        repeated_descent_allowed=False,lateral_transfer_ready=False)
    save();return destination/'worker_result.json'


def record_top_clearance_after_exit(folder,report,worker,save):
    # Called only after a clean process exit and SDK cleanup. No live session or motion follows input.
    if worker.get('hand_top_clearance_pending_after_worker_exit') is not True:return
    print('设备读取子进程已退出，连接已清理；现在填写量尺，不会再发动作或保持SDK轮询。',flush=True)
    print('仅在臂/手型/电池/板仍保持刚才末点时，量整手最低外表面高于电池顶部水平面的垂直净空；已改动或未知就q。',flush=True)
    while True:
        try:row=permission(11,'await_hand_top_clearance',input('输入实测mm（可小数，20mm及以上记录高度通过）；q=不登记高度并结束：'));break
        except ValueError as exc:print(exc,flush=True)
    gap=row.get('whole_hand_lowest_above_battery_top_mm')
    row.update(measurement_after_worker_clean_exit=True,SDK_session_alive=False,
        actual_upper_pose=worker['final_arm'],hand_shape=worker['final_hand'],
        pose_binding='operator_confirmed_unchanged_recorded_upper_no_live_reads_during_input',
        future_motion_requires_fresh_device_readback=True,lateral_transfer_ready=False,
        hand_above_target_top_confirmed=gap is not None and gap>=20)
    (folder/'hand_top_clearance.json').write_text(json.dumps(row,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    report['post_exit_top_clearance']=row;save()


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    for flag in ('execute','confirm-empty-clear-exclusive','confirm-onsite-stop','confirm-fixed-scene',
                 'confirm-fixed-camera','confirm-inert-supported-shell','enable-arm-protection','taught-only',
                 'initialize-hand-modbus','allow-supported-opening','confirm-world-x-10mm','confirm-groove-spacing-116mm',
                 'raise-for-vision-40mm','fixture-indexed-116mm'):
        p.add_argument('--'+flag,action='store_true')
    p.add_argument('--sdk-dir',type=Path,default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    p.add_argument('--output-parent',type=Path,default=ROOT/'captures')
    p.add_argument('--device',default='cuda:0')
    p.add_argument('--world-calibration',type=Path,default=ROOT/'deployment/handeye_frozen_candidate.json')
    p.add_argument('--resume-report',type=Path,
        help='Reuse this v1 clean zero-motion numerical refusal only; fresh current upper/PREPARE required')
    p.add_argument('--reuse-grasp-report',type=Path,
        help='Original completed taught descent before the reviewed contact-protection fault; pair with release report')
    p.add_argument('--reuse-release-report',type=Path,
        help='Successful supported four-channel opening from the same original descent; fresh upper/PREPARE still required')
    p.add_argument('--follow-vision-report',type=Path,
        help='Successful complete visual cycle to qualify the same shell in a complete left/right adjacent groove 116mm away, with unchanged support height')
    p.add_argument('--saved-goal-report',type=Path,
        help='Reuse a zero-write taught-goal refusal; u lifts100mm, d/g/r/u completes cycle, then measure whole-hand clearance')
    p.add_argument('--clearance-return-report',type=Path,
        help='Only continue the reviewed completed50mm taught cycle with one empty vertical lift to goal+100mm, then measure clearance')
    p.add_argument('--high-return-report',type=Path,
        help='Use the reviewed completed100mm ascent plus operator25.3mm clearance; fresh same-slot vision, descent/grip/release/empty return')
    p.add_argument('--resume-terminal-report',type=Path,
        help='Use the reviewed completed descent and existing vision; fresh terminal match, then only g/r/u')
    args=p.parse_args(argv)
    reuse_corridor=args.reuse_grasp_report is not None or args.reuse_release_report is not None
    follow_vision=args.follow_vision_report is not None
    high_return=args.high_return_report is not None
    terminal_resume=args.resume_terminal_report is not None
    if args.saved_goal_report is not None and (not args.taught_only or not args.allow_supported_opening
            or reuse_corridor or follow_vision or args.resume_report is not None
            or args.initialize_hand_modbus or args.enable_arm_protection or args.clearance_return_report is not None or high_return or terminal_resume):
        p.error('--saved-goal-report requires taught-only/supported-opening, existing accepted settings, and no other source mode; no connection')
    if args.clearance_return_report is not None and (not args.taught_only or not args.allow_supported_opening
            or reuse_corridor or follow_vision or args.resume_report is not None or args.saved_goal_report is not None
            or args.initialize_hand_modbus or args.enable_arm_protection or high_return or terminal_resume):
        p.error('--clearance-return-report requires taught-only/supported-opening and prohibits other source/configuration modes; no connection')
    if high_return and (args.taught_only or not args.allow_supported_opening or reuse_corridor or follow_vision
            or args.resume_report is not None or args.saved_goal_report is not None or args.clearance_return_report is not None
            or args.initialize_hand_modbus or args.enable_arm_protection or terminal_resume):
        p.error('--high-return-report requires visual/supported-opening and prohibits other source/configuration modes; no connection')
    if terminal_resume and (args.taught_only or not args.allow_supported_opening or reuse_corridor or follow_vision
            or args.resume_report is not None or args.saved_goal_report is not None or args.clearance_return_report is not None
            or high_return or args.initialize_hand_modbus or args.enable_arm_protection):
        p.error('--resume-terminal-report requires supported-opening and prohibits other source/configuration modes; no connection')
    if args.confirm_world_x_10mm:
        p.error('World +X 10mm has been withdrawn for this grooved fixture: it can leave the shell overhanging and change height. Use a complete adjacent groove with --confirm-groove-spacing-116mm; no connection')
    if (args.reuse_grasp_report is None)!=(args.reuse_release_report is None):
        p.error('--reuse-grasp-report and --reuse-release-report must be provided together')
    if reuse_corridor and (args.resume_report is not None or args.taught_only):
        p.error('Completed corridor reuse cannot combine with --resume-report or --taught-only')
    if follow_vision and (reuse_corridor or args.resume_report is not None or args.taught_only):
        p.error('--follow-vision-report cannot combine with completed corridor reuse, --resume-report or --taught-only')
    if follow_vision and not (args.confirm_groove_spacing_116mm and args.allow_supported_opening):
        p.error('Local visual follow requires --confirm-groove-spacing-116mm and --allow-supported-opening; no connection')
    if follow_vision and (args.initialize_hand_modbus or args.enable_arm_protection):
        p.error('Local visual follow only reads the accepted existing configuration; bus/protection setters are prohibited; no connection')
    if args.confirm_groove_spacing_116mm and not follow_vision:
        p.error('--confirm-groove-spacing-116mm applies only to --follow-vision-report')
    if args.raise_for_vision_40mm and not follow_vision:
        p.error('--raise-for-vision-40mm applies only to --follow-vision-report; no connection')
    if args.fixture_indexed_116mm and not follow_vision:
        p.error('--fixture-indexed-116mm requires --follow-vision-report with explicit 116mm spacing and supported-opening authorization; no connection')
    if not all((args.execute,args.confirm_empty_clear_exclusive,args.confirm_onsite_stop,
                args.confirm_fixed_scene,args.confirm_inert_supported_shell)):
        p.error('Requires empty/exclusive, onsite stop, fixed scene, inert board-supported shell; no connection')
    if not args.taught_only and not args.confirm_fixed_camera:
        p.error('Visual mode requires fixed camera; use --taught-only to defer camera extrinsics')
    if args.taught_only and args.resume_report:p.error('Robot restart/new teaching: --taught-only cannot reuse a previous result')
    if platform.system()!='Linux' or platform.machine()!='aarch64':p.error('Reviewed onsite Linux aarch64 only; no connection')
    try:
        hashes=installed()
        if not args.taught_only and hashlib.sha256(args.world_calibration.read_bytes()).hexdigest()!=CALIBRATION_SHA:
            raise ValueError('Calibration payload differs from frozen World preview; no connection')
    except (OSError,ValueError) as exc:p.error(str(exc))
    prefix=('hand_clearance_return_' if args.clearance_return_report is not None
        else 'taught_grasp_cycle_' if args.taught_only else 'vision_grasp_cycle_')
    folder=args.output_parent/(prefix+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True,exist_ok=False);(folder/'permissions').mkdir()
    report=dict(schema_version=1,scope='same_taught_shell_visual_descent_supported_grasp_release_empty_return',
        sdk_in_parent=False,installed_file_sha256=hashes,execute_requested=True,
        full_pregrasp_execution_ready=False,autonomous_other_targets_ready=False,offline_tests_executed=False,
        exceptions=[],worker_cleanup_timeout=False,load_lift_or_transfer_performed=False,
        onsite_hardware_stop_required_on_unexpected_contact=True,
        hand_above_target_top_confirmed=False,lateral_transfer_ready=False)
    report['explicit_supported_opening_requested']=args.allow_supported_opening
    report['raise_for_vision_40mm_requested']=args.raise_for_vision_40mm
    report['fixture_indexed_116mm_requested']=args.fixture_indexed_116mm
    report['saved_goal_automatic100_requested']=args.saved_goal_report is not None
    report['clearance_return_requested']=args.clearance_return_report is not None
    report['high_return_requested']=high_return
    report['terminal_resume_requested']=terminal_resume
    if args.taught_only:
        report.update(scope='fresh_taught_shell_descent_supported_grasp_release_empty_return',
            vision_used=False,camera_extrinsics_used_for_motion=False,
            previous_result_reused=False,fresh_start_after_restart_allowed=True)
    elif reuse_corridor:
        report.update(scope='same_saved_corridor_shell_visual_descent_supported_grasp_release_empty_return',
            same_original_unmoved_shell_only=True,completed_corridor_reuse_requested=True,
            re_teaching_or_manual_upper_required=False,automatic_other_slot_transfer=False)
    elif follow_vision:
        report.update(scope=FOLLOW_SCOPE,follow_vision_requested=True,
            measured_adjacent_groove_spacing_mm=116,independent_onsite_shift_direction_required=True,
            adjacent_groove_same_height_and_pose_required=True,re_teaching_or_manual_upper_required=False,
            camera_global_extrinsic_accuracy_certified=False,autonomous_other_targets_ready=False)
    if args.clearance_return_report is not None:
        report.update(scope='empty_vertical_clearance_from_completed_taught_cycle',fresh_start_after_restart_allowed=False)
    if high_return:
        report.update(scope='same_qualified_upper_new_wrist_visual_grasp_cycle',fresh_start_after_restart_allowed=False)
    if terminal_resume:
        report.update(scope='continued_same_slot_visual_grasp_release_empty_return',fresh_start_after_restart_allowed=False)
    def save():
        temp=folder/'parent_result.tmp'
        temp.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf8')
        temp.replace(folder/'parent_result.json')
    def abort():
        path=folder/'abort.json'
        if not path.exists():path.write_text('{"abort":true}\n',encoding='ascii')
    process=None;seen=set();native_hung=False;last_phase=None;phase_began=time.monotonic();measured_shift_mm=None;save()
    try:
        if terminal_resume:
            print('上一轮视觉绑定和约100mm下降已完成。本次重新读当前低位，g夹持 → r/n承托松指 → u空返；不重拍、不再下降、不回旧点。',flush=True)
        elif high_return:
            print('续升已完成，25.3±0.1mm净空已记录：从本次匹配的高位采当前同槽新图，b绑定 → d下降 → g夹持 → r/n松指 → u空返；不重升、不重教、不搬槽、不横移。',flush=True)
        elif args.clearance_return_report is not None:
            print('整轮抓放已完成：只从当前空手返回位续升到预抓位上方100mm，再量整手底到电池顶净空；不重抓、不横移。',flush=True)
        elif args.saved_goal_report is not None:
            print('复用已采新预抓位：u升100mm → d下降 → g夹持 → r/n承托松指 → u空返 → 实测整手底到电池顶净空；不重教、不采图、不横移。',flush=True)
        elif args.taught_only:
            print('本次新鲜示教 → 自动下降 → 夹持/松指 → 空手返回；不启用视觉外参，不使用旧结果，不带载抬升。',flush=True)
        elif reuse_corridor:
            print('原固定空壳通道：本次重新读取上方姿态 + 新YOLO/World采样 + 自动下降 + 夹持/松指 + 空手返回；不重教、不手动升高、不带载抬升。',flush=True)
        elif follow_vision:
            print('同空壳116mm相邻槽跟随：只采挪动后的图，移前基准来自成功记录；已经搬好就保持现在位置，不再搬一次。',flush=True)
            if args.fixture_indexed_116mm:
                print('已选116mm槽位定位：已量槽距和现场方向确定本次TCP横移，视觉核对新槽；保留原Y和预抓终点Z。不是任意视觉抓心定位。',flush=True)
            if args.raise_for_vision_40mm:
                print('先在独立h确认后保持空手PREPARE上升40mm，完成后才打开相机采移后图；电池不再移动。',flush=True)
        else:
            print('同一固定空壳：本次示教绑定 + YOLO/World定位 + 自动下降 + 夹持/松指 + 空手返回；不带载抬升。',flush=True)
        print('启动时空手悬空、退出其他示教/控制。只在明确示教提示期间允许点动；其余时间独占控制。',flush=True)
        print('意外接触/滑落/仍运动：立即现场硬件停止并安全解除受力。q/退出/ACK不是急停或解除扭矩。',flush=True)
        resumed=prepare_resume(args.resume_report,folder,report,save) if args.resume_report else None
        if reuse_corridor:capture_corridor_sources(args,folder,report,save)
        follow_source=capture_follow_source(args.follow_vision_report,folder,report,save) if follow_vision else None
        saved_goal_source=capture_saved_goal_source(args.saved_goal_report,folder,report,save) if args.saved_goal_report is not None else None
        clearance_source=capture_clearance_source(args.clearance_return_report,folder,report,save) if args.clearance_return_report is not None else None
        high_return_source=capture_high_return_source(args.high_return_report,folder,report,save) if high_return else None
        terminal_source=capture_terminal_resume_source(args.resume_terminal_report,folder,report,save) if terminal_resume else None
        if resumed:print('续跑：不重拍、不重教；自动核对当前位置，d才允许下降。',flush=True)
        command=[sys.executable,str(ROOT/'deployment/vision_cycle_worker.py'),'--folder',str(folder),
            '--sdk-dir',str(args.sdk_dir),'--execute']
        if resumed:command.extend(['--resume-report',str(resumed)])
        if saved_goal_source:command.extend(['--saved-goal-report',str(saved_goal_source)])
        if clearance_source:command.extend(['--clearance-return-report',str(clearance_source)])
        if high_return_source:command.extend(['--high-return-report',str(high_return_source)])
        if terminal_source:command.extend(['--resume-terminal-report',str(terminal_source)])
        if reuse_corridor:
            command.extend(['--reuse-grasp-report',str(folder/'sources/grasp_worker_result.json'),
                '--reuse-release-report',str(folder/'sources/release_worker_result.json')])
        if follow_source:command.extend(['--follow-vision-report',str(follow_source),'--confirm-groove-spacing-116mm'])
        if args.raise_for_vision_40mm:command.append('--raise-for-vision-40mm')
        if args.fixture_indexed_116mm:command.append('--fixture-indexed-116mm')
        if args.taught_only:command.append('--taught-only')
        if args.initialize_hand_modbus:command.append('--initialize-hand-modbus')
        if args.allow_supported_opening:command.append('--allow-supported-opening')
        if args.enable_arm_protection:command.append('--enable-arm-protection')
        report['worker_process']=dict(command=command,stdin='DEVNULL',close_fds=True,SDK_in_parent=False)
        with (folder/'process_stdout.log').open('wb') as out,(folder/'process_stderr.log').open('wb') as err:
            process=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=out,stderr=err,close_fds=True)
            while process.poll() is None:
                path=folder/'worker_result.json'
                if path.exists():
                    state=json.loads(path.read_text(encoding='utf8'));phase=state.get('phase');index=state.get('permission_index')
                    if phase!=last_phase:
                        last_phase=phase;phase_began=time.monotonic()
                        if not str(phase).startswith('await_'):print('PHASE:',phase,flush=True)
                    limit={'preflight':90,'preflight_goal':90,'preflight_upper':90,
                        'preflight_descending':90,'preflight_grasp':90,'preflight_release':90,
                        'preflight_aligning_visual_XY':90,
                        'preflight_camera_clearance_lift':90,
                        'preflight_raising_camera_clearance':90,
                        'preflight_lifting_saved_goal_100mm':90,'preflight_lifting_empty_clearance':90,
                        'preflight_returning_empty':90,'preparing_arm_protection':30,
                        'initializing_hand_modbus':90,
                        'preparing_empty_hand':60,'grasping':90,'releasing':60,'completed':15,'fault':15}.get(phase)
                    if phase in ('final_cycle_preflight','preflight_clearance_measurement'):limit=90
                    if phase in ('aligning_visual_XY','raising_camera_clearance','lifting_saved_goal_100mm','lifting_empty_clearance','descending','returning_empty'):limit=state.get('plan',{}).get('arm_timeout_seconds',60)+10
                    if limit is not None and time.monotonic()-phase_began>limit:
                        raise TimeoutError('Worker phase deadline: '+str(phase)+'; no retry/automatic return')
                    if phase in {v[0] for v in PHASES.values()} and index not in seen:
                        if index not in PHASES or phase!=PHASES[index][0]:raise RuntimeError('Worker phase/index mismatch')
                        seen.add(index);target=None;direction_cancelled=False
                        if index==3:
                            target=capture_vision(args,folder,report,save)
                            if target['class_name']!='black_cyl_battery':raise RuntimeError('Select black cylindrical test shell for this hand profile')
                            if follow_vision:
                                while True:
                                    direction=input('现场独立确认：从原槽到当前槽，空壳沿机器人哪侧移动116mm？l=机器人左/World +X；r=机器人右/World -X；q=取消（不要根据视觉数值猜方向）：').strip().lower()
                                    if direction=='q':direction_cancelled=True;break
                                    if direction in ('l','r'):
                                        measured_shift_mm=116 if direction=='l' else -116
                                        report['onsite_measured_world_X_shift_mm']=measured_shift_mm
                                        report['onsite_direction_origin']='independent_operator_choice_after_fresh_vision_not_derived_from_vision'
                                        save();break
                                    print('只输入l、r或q；必须是沿World X的一排相邻完整槽。',flush=True)
                        if index==4 and follow_vision:
                            if type(measured_shift_mm) is not int or measured_shift_mm not in (-116,116):
                                raise RuntimeError('No independently selected adjacent-groove direction; descent refused')
                            print('本轮已绑定现场方向：',('机器人左 / World +X' if measured_shift_mm==116 else '机器人右 / World -X'),measured_shift_mm,'mm；不重新选方向。',flush=True)
                        prompt=operator_prompt(index,state)
                        while True:
                            answer='q' if direction_cancelled else input(prompt)
                            try:
                                row=permission(index,phase,answer,target,reuse_corridor,follow_vision,measured_shift_mm,high_return or terminal_resume)
                                if terminal_resume and index==5:
                                    row['same_high_return_shell_scene_and_clearance_confirmed']=row['value']!='q'
                                break
                            except ValueError as exc:print(exc,flush=True)
                        current=json.loads(path.read_text(encoding='utf8'))
                        if process.poll() is not None or current.get('phase')!=phase or current.get('permission_index')!=index:
                            raise RuntimeError('Worker changed/aborted during input; stale permission refused')
                        report.setdefault('operator_inputs',[]).append(row);save();write_permission(folder,row)
                elif time.monotonic()-phase_began>30:raise TimeoutError('No worker startup report; no retry')
                time.sleep(.2)
            report['worker_return_code']=process.returncode
        completed_path=folder/'worker_result.json'
        if process.returncode==0 and completed_path.exists():
            completed_worker=json.loads(completed_path.read_text(encoding='utf8'))
            if not completed_worker.get('exceptions') and not completed_worker.get('cleanup_errors'):
                record_top_clearance_after_exit(folder,report,completed_worker,save)
    except (Exception,KeyboardInterrupt) as exc:
        report['exceptions'].append(dict(error=repr(exc)));abort()
        print('异常：立即现场硬件停止并安全解除受力；已禁止后续动作，不自动张开/回臂。',flush=True)
        if process is not None and process.poll() is None:
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    native_hung=True;report['worker_cleanup_timeout']=True
                    print('SDKworker未确认退出。须现场硬件停止；杀进程不证明停机。',flush=True)
    finally:
        if process is not None:report['worker_return_code']=process.poll()
        path=folder/'worker_result.json'
        if path.exists():report['worker_result']=json.loads(path.read_text(encoding='utf8'))
        outcome=report.get('worker_result',{}).get('outcome','worker_result_missing')
        measured=report.get('post_exit_top_clearance')
        if outcome==CLEARANCE_PENDING and measured is not None:
            outcome=(CLEARANCE_UNMEASURED if measured['value']=='q'
                else CLEARANCE_SUCCESS if measured['hand_above_target_top_confirmed'] else CLEARANCE_INSUFFICIENT)
        if native_hung:outcome='SDK_worker_unresponsive_hardware_stop_required'
        report['outcome']=outcome
        report['hand_above_target_top_confirmed']=(measured['hand_above_target_top_confirmed'] if measured is not None
            else report.get('worker_result',{}).get('hand_above_target_top_confirmed',False))
        report['lateral_transfer_ready']=False;save()
        bundle=folder.with_suffix('.zip')
        with zipfile.ZipFile(bundle,'x',zipfile.ZIP_DEFLATED) as z:
            for path in sorted(folder.rglob('*')):
                if path.is_file():z.write(path,str(Path(folder.name)/path.relative_to(folder)))
        print('SAVED:',bundle,'OUTCOME:',outcome,flush=True)
        errors=report.get('worker_result',{}).get('exceptions',[])
        if errors:print('ERROR:',errors[-1]['error'],flush=True)
        if outcome in (CLEARANCE_SUCCESS,CLEARANCE_INSUFFICIENT,CLEARANCE_PENDING,CLEARANCE_UNMEASURED):
            print('空手续升已完成；整手底高于电池顶的实测净空：',
                (measured or report['worker_result']).get('whole_hand_lowest_above_battery_top_mm'),'mm；高度通过：',
                report['hand_above_target_top_confirmed'],'。未授权横移；本次没有重做抓取。',flush=True)
        elif outcome in SUCCESS_OUTCOMES:
            print('流程完成；现场观察夹持成功：',report['worker_result'].get('grip_observed_by_operator'),
                '。未做承重/带载抬升。',flush=True)
            if outcome==FOLLOW_SUCCESS:
                print('本轮已返回上方点；尚未证明整手越过电池顶或横移通道安全。下一轮须重新读取当前位置。',flush=True)
            print('整手高于电池顶的现场测量通过：',report['hand_above_target_top_confirmed'],'；未授权横移。',flush=True)
        else:print('流程未完成。禁止自动重试；急停及解除手受力须由现场处理。',flush=True)
        if native_hung:print('ZIP为日志快照；原目录保留，worker没有确认停止。',flush=True)
    return 0 if report['outcome'] in SUCCESS_OUTCOMES and not report['exceptions'] else 1


if __name__=='__main__':raise SystemExit(main())
