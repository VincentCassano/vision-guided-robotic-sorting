"""Source-qualified current held drop, one visual binding and fresh-start cycles."""
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import zipfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/battery_grasp/src'))


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    from known_cylinder_records import dump
    from visual_pick_sort_v16_source import (verify_delivery,accepted_success,accepted_anchor,
        make_anchor,target_candidate,SUCCESS_SHA,MANIFEST)
    from visual_pick_sort_v13_source import read_point_archive
    folder=ROOT/'captures'/datetime.now().strftime('visual_pick_sort_v16_%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True,exist_ok=False)
    pointer=ROOT/'calibration/visual_pick_sort_v14_anchor.json'
    anchor=point=candidate=selected_archive=None
    empty_confirmed=False;process=None;must_quit=False
    report=dict(version=16,scope='actual044001_new_grip_fresh_visual_single_black_shell_sort',
        outcome='started',success_source_sha256=SUCCESS_SHA,attempts=[],captures=[],exceptions=[],
        parent_SDK_loaded=False,controller_configuration_writes=0,offline_project_tests_executed=False,
        current_hand_state_must_be_confirmed=True,complete_visual_pick_sort_accepted=False,
        arbitrary_start_pose_universally_reachable_claimed=False,execution_ready=False)

    def save():dump(folder/'result.json',report)

    def child(command,output=None):
        nonlocal process
        process=subprocess.Popen(command,cwd=str(ROOT))
        try:
            while process.poll() is None:
                try:process.wait(timeout=.25)
                except subprocess.TimeoutExpired:continue
                except KeyboardInterrupt:
                    if output is not None:
                        try:dump(output.parent/'abort.json',dict(operator_requested_stop=True))
                        except Exception as exc:print('停止请求文件未写入：'+str(exc),flush=True)
                        print('已请求中止，等待动作进程关闭；不启动第二个控制。',flush=True)
                    else:print('等采图进程退出后再继续；未启动臂手控制。',flush=True)
            return process.returncode
        finally:
            if process is not None and process.poll() is not None:process=None

    def capture(role,surface='recess'):
        before=set((ROOT/'captures').glob('visual_pick_point_v16_'+role+'_*.zip'))
        command=[sys.executable,str(ROOT/'deployment/visual_pick_sort_v16_vision.py'),
            '--role',role,'--support-surface',surface,'--position-session',str(ROOT/'calibration/position_rgbd_20261020_001259_216986'),
            '--anchor',str(ROOT/'deployment/visual_reference_201634/task_anchor.json'),
            '--confirm-fixed-camera','--confirm-measured-shell-only','--confirm-unchanged-setup',
            '--confirm-upright-shell-and-selected-support','--finish-after-first-reference',
            '--inference-python','/usr/bin/python3','--device','auto']
        code=child(command)
        added=set((ROOT/'captures').glob('visual_pick_point_v16_'+role+'_*.zip'))-before
        if code or len(added)!=1:raise ValueError('采图尚未完成，记录保留；可重新选取，不发臂动作')
        source=added.pop();value,evidence=read_point_archive(source)
        copied=folder/('%s_%03d.zip'%(role,len(report['captures'])+1));copied.write_bytes(source.read_bytes())
        report['captures'].append(dict(role=role,file=copied.name,sha256=sha(copied),evidence=evidence))
        save();return value,copied

    def bind_reference():
        nonlocal anchor,point,candidate,selected_archive
        if not empty_confirmed:raise ValueError('当前仍可能持物；先h正常投放，或e明确手已空')
        print('只绑定一次：同一空壳放回刚才成功抓取前的原位，落底。头/板/安装不动。',flush=True)
        if pointer.exists():
            pointer.replace(folder/('superseded_anchor_'+datetime.now().strftime('%H%M%S_%f')+'.json'))
        anchor=None;report.pop('anchor_sha256',None)
        point=candidate=selected_archive=None
        save()
        reference,source=capture('reference')
        created=make_anchor(reference,True)
        dump(folder/'new_visual_anchor.json',created)
        pointer.parent.mkdir(exist_ok=True)
        dump(pointer,created);anchor=created
        report.update(outcome='actual_grip_visual_reference_bound',anchor_sha256=sha(pointer),
            reference_capture=source.name);save()
        print('新手势/腕姿已绑定视觉。槽内用s，板顶用t，采图S后再g整轮。',flush=True)

    def select_target(surface='recess'):
        nonlocal point,candidate,selected_archive
        if not empty_confirmed:raise ValueError('先确认手为空，不能持物开始新抓取')
        if anchor is None:raise ValueError('先b绑定成功抓取前的原位置')
        point=candidate=selected_archive=None
        print(('槽内目标：原支撑高度。' if surface=='recess' else '板顶目标：支撑比槽底高9.6mm，重新计算每帧视线交点XYZ。')+'窗口点选后按S；之后空壳别再动。',flush=True)
        fresh,archive=capture('target',surface);planned=target_candidate(fresh,anchor)
        point=fresh;candidate=planned;selected_archive=archive
        dump(folder/'latest_live_target.json',dict(point_record=point,candidate=candidate,vision_file=archive.name))
        print('已保存本次视觉目标，预抓XYZ(mm)：'+str([round(v*1000,3) for v in planned['pregrasp_tcp_candidate_m_rad'][:3]]),flush=True)
        print('g：空手高位→对准→下降到位→整组预抓→整组夹持→提180mm→黑框整组松开。',flush=True)

    def supported_open():
        nonlocal point,candidate,selected_archive,empty_confirmed
        point=candidate=selected_archive=None;empty_confirmed=False
        attempt=folder/('supported_open_%03d'%(len(report.get('supported_releases',[]))+1))
        attempt.mkdir();output=attempt/'result.json';save()
        code=child([sys.executable,str(ROOT/'deployment/run_supported_open_v16.py'),
            '--output',str(output),'--confirm-supported-stationary-clear-exclusive'],output)
        result=json.loads(output.read_bytes()) if output.exists() else {}
        if code or result.get('exceptions'):
            for row in result.get('exceptions',[]):print('松指未完成：'+str(row.get('error',row)),flush=True)
        report.setdefault('supported_releases',[]).append(dict(exit_code=code,
            result_file=str(output.relative_to(folder)),result_sha256=sha(output) if output.exists() else None,
            outcome=result.get('outcome')))
        save()
        print('张开结果：'+str(result.get('outcome'))+'；未移臂。实际松开并取出空壳后再e；目标回读不代表已经松开。',flush=True)

    def action(mode):
        nonlocal point,candidate,selected_archive,empty_confirmed,must_quit
        if mode!='deliver_held' and not empty_confirmed:
            raise ValueError('当前手是否为空未确认；不能开始空手路径')
        if mode=='visual_cycle' and (anchor is None or candidate is None):
            raise ValueError('先b完成一次绑定，再s采本轮目标；不使用旧目标补位')
        attempt=folder/('cycle_%03d'%(len(report['attempts'])+1));attempt.mkdir()
        permit=dict(exclusive=True,onsite_stop=True,inert_shell=True,fixed_head_board_mount=True,
            whole_route_and_sweep_clear=True)
        if mode=='deliver_held':permit.update(same_current_held_shell=True,no_release_move_or_restart_since_success=True)
        else:permit['hand_empty']=True
        request=dict(mode=mode,success_source_sha256=SUCCESS_SHA,manifest_sha256=sha(MANIFEST),
            scene_permission=permit,timestamp_utc=datetime.now(timezone.utc).isoformat())
        if mode=='visual_cycle':
            permit.update(target_unchanged_since_capture=True,continuous_pick_sort_authorized=True)
            request.update(anchor=anchor,point_record=point,candidate=candidate,
                vision_archive_sha256=sha(selected_archive))
        request_file=attempt/'request.json';output=attempt/'worker_result.json';dump(request_file,request)
        item=dict(mode=mode,folder=attempt.name,request_sha256=sha(request_file))
        report['attempts'].append(item)
        # Every accepted action consumes its current scene. Unknown/faulted
        # object state cannot silently become the next empty-start permission.
        point=candidate=selected_archive=None;empty_confirmed=False;save()
        code=child([sys.executable,str(ROOT/'deployment/visual_pick_sort_v16_worker.py'),
            '--request',str(request_file),'--output',str(output),'--manifest',str(MANIFEST)],output)
        result=json.loads(output.read_bytes()) if output.exists() else {}
        item.update(exit_code=code,result_file=str(output.relative_to(folder)),
            result_sha256=sha(output) if output.exists() else None,outcome=result.get('outcome'))
        save()
        if code or result.get('phase')!='completed' or result.get('exceptions') or result.get('cleanup_errors'):
            reasons=[r.get('error',str(r)) for r in result.get('exceptions',[])+result.get('cleanup_errors',[]) if isinstance(r,dict)]
            print('本轮未完成：'+'；'.join(reasons or [str(result.get('outcome','结果未返回'))]),flush=True)
            print('界面保留，不自动重发、张开或回臂；先现场确认。',flush=True)
            report['outcome']='worker_not_completed_no_automatic_retry';save();return
        if mode=='park_empty':
            empty_confirmed=True;report['outcome']='empty_hand_parked_at_source_upper';save()
            print('已空手张开并到达实测高位；未执行预抓，可采图。',flush=True);return
        answer=input('y确实入黑框且手已空 | f漏抓/未入框但手已空 | q仍持物或不确定：').strip().lower()
        item['operator_result']=answer;empty_confirmed=answer in ('y','f')
        item['actual_observed_bin_success']=answer=='y'
        if mode=='visual_cycle':
            report['complete_visual_pick_sort_accepted']=report['complete_visual_pick_sort_accepted'] or answer=='y'
            report['outcome']='visual_pick_sort_observed_success' if answer=='y' else 'visual_cycle_result_not_success'
        else:report['outcome']='initial_held_shell_delivered' if answer=='y' else 'initial_held_delivery_not_confirmed'
        save()
        if answer not in ('y','f'):must_quit=True
        elif anchor is None:print('下一步：将同一空壳放回本次成功抓取前的原位，b只绑定一次。',flush=True)
        else:print('本轮已记录。下一处槽内用s、板顶用t，采图S后再g；可从当前空手臂位开始。',flush=True)

    try:
        report['checked_files']=len(verify_delivery());success=accepted_success()
        report['accepted_actual_pickup_mm']=success['actual_lift_world_mm']
        if pointer.exists():
            try:
                anchor=accepted_anchor(json.loads(pointer.read_bytes()));report['loaded_anchor_sha256']=sha(pointer)
            except Exception as exc:
                anchor=None;report['prior_binding_unavailable']=repr(exc)
                print('原视觉绑定不可用；手中空壳仍可h投放，之后b重新绑定。',flush=True)
        save()
        print('v16：复用成功腕姿/绑定；s采槽内，t采亚克力板顶（实测高9.6mm），g完整取放。',flush=True)
        print('对准并下降到位后才PRE6预抓→GRIP3夹持→提180mm→转运→OPEN5同时张开。',flush=True)
        print('h/p/g同时确认：退出其他控制，现场守急停，整条臂/手/相机/线缆/持物及回转扫掠无障碍，预计净空≥20mm。',flush=True)
        print('当前受承托仍夹着：臂已停稳、张开扫掠清空时o整组松指；实际手空后e。无需重做b。',flush=True)
        print('槽内e→s/S→g；槽外板顶e→t/S→g。高度由所选支撑面给定，尚非自动测任意高度。',flush=True)
        print('空手MoveJ到高位期间可同时整组张开；已张开则跳过。高位与下降期间保持空手张开，不提前预抓。',flush=True)
        while not must_quit:
            try:
                answer=input('o承托下松指 | e手已空 | s槽内采图 | t板顶采图 | g取放 | p高位 | b重绑 | q退出：').strip().lower()
                if answer=='q':break
                if answer=='h':action('deliver_held')
                elif answer=='e':
                    empty_confirmed=True;point=candidate=selected_archive=None
                    report.setdefault('operator_empty_confirmations',[]).append(datetime.now(timezone.utc).isoformat());save()
                    print('已记录手为空；未发臂手动作。',flush=True)
                elif answer=='b':bind_reference()
                elif answer=='s':select_target('recess')
                elif answer=='t':select_target('board_top')
                elif answer=='o':supported_open()
                elif answer=='g':action('visual_cycle')
                elif answer=='p':action('park_empty')
            except KeyboardInterrupt:print('界面保留，不自动重发。',flush=True)
            except EOFError:break
            except Exception as exc:
                report['exceptions'].append(dict(error=repr(exc),timestamp_utc=datetime.now(timezone.utc).isoformat()))
                save();print(str(exc)+'；当前记录保留。',flush=True)
    except Exception as exc:
        report['exceptions'].append(dict(error=repr(exc)));report['outcome']='startup_not_completed';save()
        print('未开始运动：'+str(exc),flush=True)
    finally:
        save()
        hashes={p.relative_to(folder).as_posix():sha(p) for p in folder.rglob('*')
            if p.is_file() and p!=folder/'source_hashes.json'}
        dump(folder/'source_hashes.json',hashes)
        archive=folder.with_suffix('.zip')
        with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as out:
            for p in sorted(folder.rglob('*')):
                if p.is_file():out.write(p,folder.name+'/'+p.relative_to(folder).as_posix())
        print('SAVED:',archive,'SHA256:',sha(archive),flush=True)


if __name__=='__main__':main()
