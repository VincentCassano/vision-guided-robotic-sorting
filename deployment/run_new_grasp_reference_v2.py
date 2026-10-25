"""Record a newly taught wrist and a real empty-hand vertical raise before vision.

No automatic arm motion. SDK exists only in the existing read-only child.
The optional hand console acts only on explicit operator commands.
"""
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/battery_grasp/src'))
MANIFEST=ROOT/'deployment/new-grasp-reference-v2-files.sha256'
PRE=[35000,7500,9480,30000,65535,65535]
RETIRED=ROOT/'calibration/new_wrist_reference_pending.json'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    from known_cylinder_records import dump
    from new_grasp_task_anchor_v2 import validate_lift_reference,make_lift_anchor
    folder=ROOT/'captures'/datetime.now().strftime('new_grasp_reference_v2_%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True,exist_ok=False)
    report=dict(version=2,scope='new_wrist_and_real_empty_vertical_raise_then_same_place_vision',
        outcome='recording',parent_SDK_loaded=False,parent_arm_motion_writes=0,
        parent_arm_stop_writes=0,parent_hand_target_writes=0,controller_configuration_writes=0,
        optional_hand_console_explicit_targets_only=True,execution_ready=False,
        exceptions=[],read_attempts=[],hand_consoles=[],object_pickup_verified=False,
        complete_visual_pick_sort_accepted=False,offline_project_tests_executed=False)
    current=upper=lift=None
    child=None

    def save():dump(folder/'result.json',report)

    def retire(reason):
        RETIRED.parent.mkdir(parents=True,exist_ok=True)
        dump(RETIRED,dict(reason=reason,session=str(folder),timestamp_utc=datetime.now(timezone.utc).isoformat(),
            old_reference_retired=True,motion_integration_pending=True,execution_ready=False))

    def clear_binding():
        old=folder/'new_task_anchor.json'
        if old.exists():old.replace(folder/datetime.now().strftime('superseded_%H%M%S_%f_anchor.json'))
        report.pop('task_anchor_sha256',None)
        retire('operator_reteaching_or_resampling_reference')

    def invalidate():
        nonlocal current,upper,lift
        current=upper=lift=None
        clear_binding()
        for key in ('lift_reference','seated_reference_file','upper_reference_file'):
            report.pop(key,None)
        report['outcome']='needs_new_seated_and_upper_reference';retire('operator_reteaching_wrist');save()

    def run_child(command):
        nonlocal child
        child=subprocess.Popen(command,cwd=str(ROOT))
        try:return child.wait()
        except KeyboardInterrupt:
            if child.poll() is None:child.terminate()
            try:child.wait(timeout=10)
            except subprocess.TimeoutExpired:child.kill();child.wait()
            report.setdefault('interrupted_children',[]).append(dict(command=command,
                exit_code=child.returncode,SDK_cleanup_not_assumed=True))
            raise
        finally:child=None

    def hand_read():
        from manual_hand_bus_v2 import ManualHandBus
        def append(row):
            with (folder/'hand_wire.jsonl').open('a',encoding='utf8') as out:
                out.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
        bus=ManualHandBus(append)
        try:return bus.connect_read()
        finally:bus.close()

    def require_pre(row):
        if (row.get('ok') is not True or row.get('snapshot_stale') is not False
                or row.get('needs_reconnect') or not row.get('snapshot')):
            raise ValueError('手部读数未完整取得，界面保留；停稳后可重读。')
        hand=row['snapshot']
        if len(hand.get('targets',[]))!=6 or any(abs(a-b)>2 for a,b in zip(hand['targets'],PRE)):
            raise ValueError('当前不是指定预抓目标。仅此时需要h → pre → q；不用重新调数字。')
        if len(hand.get('status',[]))!=6 or any(v not in (0,1,2,5) for v in hand['status']):
            raise ValueError('手部存在真实保护，原始读数已保存，界面保留。')
        return hand

    def read_pose(role):
        print('正在只读%s，读取期间保持静止；本程序不发臂动作。'%(
            '抓前点' if role=='seated_pregrasp' else '实际升后点'),flush=True)
        attempt=folder/('%s_%03d.json'%(role,len(report['read_attempts'])+1))
        item=dict(role=role,file=attempt.name,timestamp_utc=datetime.now(timezone.utc).isoformat())
        report['read_attempts'].append(item);save()
        before=hand_read();item['hand_before']=before;save();require_pre(before)
        code=run_child([sys.executable,str(ROOT/'deployment/new_grasp_reference_worker_v1.py'),
            '--output',str(attempt)])
        if not attempt.exists():raise ValueError('只读子进程没有返回记录，未发动作，界面保留。')
        arm=json.loads(attempt.read_bytes());item.update(worker_exit_code=code,sha256=sha(attempt));save()
        if code or not arm.get('reference_qualified') or not arm.get('SDK_cleanup_finished'):
            reasons=[e.get('error',str(e)) for e in arm.get('exceptions',[]) if isinstance(e,dict)]
            raise ValueError('臂位只读未合格：'+'；'.join(reasons or ['详见记录，SDK退出状态需现场确认']))
        q=arm['reference_arm']['joints'];limits=arm['reference_arm']['limits']
        for index,(value,low,high) in enumerate(zip(q,limits['minimum'],limits['maximum'])):
            margin=min(value-low,high-value)
            if margin<3:
                raise ValueError('J%d当前%.3f°、限位余量%.3f°；请改腕姿后再记录。'%(index+1,value,margin))
        after=hand_read();item['hand_after']=after;save();require_pre(after)
        if any(abs(a-b)>2 for a,b in zip(before['snapshot']['targets'],after['snapshot']['targets'])):
            raise ValueError('读取期间手目标变了，本次不作参考，界面保留。')
        item['recorded']=True;save()
        return dict(arm=arm,hand=after,file=attempt,sha256=sha(attempt))

    def record_start():
        nonlocal current
        invalidate();current=read_pose('seated_pregrasp')
        p=current['arm']['reference_arm']['pose']
        report['seated_reference_file']=current['file'].name;save()
        print('抓前点已保存，SDK已关闭。World XYZ(mm)：',[round(v*1000,3) for v in p[:3]],flush=True)
        print('现在用示教器保持X/Y和腕姿，把World Z从%.3fmm升到%.3fmm（+150mm）。空手升，空壳留原位。'%(p[2]*1000,p[2]*1000+150),flush=True)
        print('升得上去：退出示教、停稳后u记录。升不上去：沿原路返回，重新摆腕，再r；程序不自动回臂。',flush=True)

    def record_upper():
        nonlocal upper,lift
        if current is None:raise ValueError('先摆好新的抓前腕姿，r记录。')
        upper=lift=None;clear_binding()
        report.pop('lift_reference',None);report.pop('upper_reference_file',None)
        report['outcome']='needs_current_upper_reference';save()
        upper=read_pose('upper_after_manual_empty_raise')
        lift=validate_lift_reference(current['arm'],current['hand'],upper['arm'],upper['hand'])
        report['lift_reference']=lift;report['upper_reference_file']=upper['file'].name
        report['outcome']='seated_and_actual_empty_raise_recorded';save()
        print('抓前点与实际空手升后点已保存，SDK已关闭。保持空壳/头/板不动，v采原位空壳视觉。',flush=True)

    def capture_visual():
        if current is None or upper is None or lift is None:
            raise ValueError('先r记抓前点，再实际空手上升并u记升后点；不只保存静态腕姿。')
        clear_binding();report['outcome']='needs_current_visual_reference';save()
        before=set((ROOT/'captures').glob('new_grasp_reference_vision_*.zip'))
        code=run_child([sys.executable,str(ROOT/'deployment/run_new_grasp_reference_vision_v1.py'),
            '--position-session',str(ROOT/'calibration/position_rgbd_20261020_001259_216986'),
            '--anchor',str(ROOT/'deployment/visual_reference_201634/task_anchor.json'),
            '--confirm-fixed-camera','--confirm-measured-shell-only','--confirm-unchanged-setup',
            '--confirm-upright-equal-height-recesses','--finish-after-first-reference',
            '--inference-python','/usr/bin/python3','--device','auto'])
        created=set((ROOT/'captures').glob('new_grasp_reference_vision_*.zip'))-before
        if code or len(created)!=1:raise ValueError('图像尚未保存；腕姿记录保留，可v重采，未发动作。')
        source=created.pop();copy=folder/('vision_reference_%03d.zip'%(len(report.get('vision_sources',[]))+1))
        copy.write_bytes(source.read_bytes())
        report.setdefault('vision_sources',[]).append(dict(file=copy.name,sha256=sha(copy)));save()
        with zipfile.ZipFile(copy) as archive:
            if archive.testzip() is not None:raise ValueError('视觉ZIP不完整，记录保留。')
            members=archive.namelist()
            if len(members)!=len(set(members)):raise ValueError('视觉ZIP存在重复成员。')
            manifests=[n for n in members if n.endswith('/source_hashes.json')]
            if len(manifests)!=1:raise ValueError('视觉SHA清单不唯一。')
            prefix=manifests[0][:-len('source_hashes.json')]
            for name,digest in json.loads(archive.read(manifests[0])).items():
                if '/' in name or '\\' in name or hashlib.sha256(archive.read(prefix+name)).hexdigest()!=digest:
                    raise ValueError('视觉文件SHA不符：'+name)
            point=json.loads(archive.read(prefix+'latest_reference_point.json'))
            visual_report=json.loads(archive.read(prefix+'result.json'))
            if visual_report.get('error') or visual_report.get('cleanup_errors'):
                raise ValueError('视觉进程异常或清理未完成；参考保留。')
            runtime=visual_report.get('runtime_source_sha256')
            if not isinstance(runtime,dict) or not runtime:raise ValueError('视觉运行源码SHA缺失。')
            for name,digest in runtime.items():
                if sha(ROOT/'deployment'/name)!=digest:raise ValueError('当前视觉源码不符：'+name)
        recipe_path=ROOT/'deployment/black_shell_new_grasp_recipe_20261023.json'
        anchor=make_lift_anchor(current['arm'],current['hand'],upper['arm'],upper['hand'],point,
            json.loads(recipe_path.read_bytes()),dict(arm_reference_file=current['file'].name,
                arm_reference_sha256=current['sha256'],upper_arm_reference_file=upper['file'].name,
                upper_arm_reference_sha256=upper['sha256'],vision_archive_file=copy.name,
                vision_archive_sha256=sha(copy),recipe_sha256=sha(recipe_path)))
        prior=folder/'new_task_anchor.json'
        if prior.exists():prior.replace(folder/datetime.now().strftime('superseded_%H%M%S_%f_anchor.json'))
        dump(prior,anchor)
        destination=ROOT/'calibration/black_shell_new_grasp_task_anchor.json'
        if destination.exists():
            backup=folder/datetime.now().strftime('previous_calibration_pointer_%H%M%S_%f.json')
            backup.write_bytes(destination.read_bytes())
        dump(destination,anchor)
        marker=json.loads(RETIRED.read_bytes())
        marker.update(new_anchor_sha256=sha(prior),new_reference_recorded=True,motion_integration_pending=True)
        dump(RETIRED,marker)
        report.update(outcome='new_wrist_real_empty_raise_and_visual_reference_recorded',
            task_anchor_sha256=sha(prior),execution_ready=False,motion_integration_pending=True);save()
        print('新抓前点、实际空手升后点及原位视觉已绑定。q保存一个ZIP；旧腕姿已停用，新参考尚需接入动作。',flush=True)

    def hand_console():
        invalidate()
        before=set((ROOT/'captures').glob('hand_thumb_tuner_v2_*'))
        code=run_child([sys.executable,str(ROOT/'deployment/run_hand_thumb_tuner_v2.py'),'--execute'])
        created=[p for p in set((ROOT/'captures').glob('hand_thumb_tuner_v2_*'))-before if p.is_dir()]
        row=dict(exit_code=code,recorded_folders=[])
        for source in created:
            dest=folder/('hand_console_%03d'%(len(report['hand_consoles'])+1))/source.name
            shutil.copytree(source,dest);row['recorded_folders'].append(dest.relative_to(folder).as_posix())
        report['hand_consoles'].append(row);save()
        print('调手连接已关闭；现在摆腕，r记录抓前点。手型没变时不用再h/pre。',flush=True)

    try:
        checked=0
        for line in MANIFEST.read_text(encoding='ascii').splitlines():
            digest,name=line.split('  ',1);p=(ROOT/name).resolve()
            if ROOT not in p.parents or sha(p)!=digest:raise ValueError('交付文件不符：'+name)
            checked+=1
        if not checked:raise ValueError('交付清单为空。')
        report['checked_files']=checked
        if RETIRED.exists():(folder/'previous_retirement_marker.json').write_bytes(RETIRED.read_bytes())
        retire('old_wrist_could_not_complete_fixed_world_vertical_raise');save()
        print('重新记录：r抓前点 → 现场空手World Z+150mm → u升后点 → v拍原位空壳 → q统一ZIP。',flush=True)
        print('手型保持PRE；只有目标已改变才h→pre→q。程序不自动移臂，不自动提起或分拣。',flush=True)
        print('先用示教器重新调整腕姿/位置；预抓围住同一落底空壳，空壳留在原位。退出示教、停稳撤手后r。',flush=True)
        while True:
            try:
                answer=input('r抓前 | u实际空手升后 | v原位视觉 | h仅需恢复手型时 | q保存退出：').strip().lower()
                if answer=='q':break
                if answer=='r':record_start()
                elif answer=='u':record_upper()
                elif answer=='v':capture_visual()
                elif answer=='h':hand_console()
                else:print('输入r、u、v、h或q；界面保留。',flush=True)
            except KeyboardInterrupt:print('本次已中断，界面保留；未自动重试。',flush=True)
            except EOFError:break
            except Exception as exc:
                report['exceptions'].append(dict(error=repr(exc),timestamp_utc=datetime.now(timezone.utc).isoformat()))
                save();print(str(exc),flush=True)
    except Exception as exc:
        report['exceptions'].append(dict(error=repr(exc)));report['outcome']='startup_not_completed';save()
        print(str(exc),flush=True)
    finally:
        report['execution_ready']=False;save()
        hashes={p.relative_to(folder).as_posix():sha(p) for p in folder.rglob('*')
            if p.is_file() and p.name!='source_hashes.json'}
        dump(folder/'source_hashes.json',hashes)
        archive=folder.with_suffix('.zip')
        with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as out:
            for p in sorted(folder.rglob('*')):
                if p.is_file():out.write(p,folder.name+'/'+p.relative_to(folder).as_posix())
        print('SAVED:',archive,'SHA256:',sha(archive),flush=True)


if __name__=='__main__':main()
