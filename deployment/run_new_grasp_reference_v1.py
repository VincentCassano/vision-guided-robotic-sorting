"""Record one newly successful seated pregrasp wrist and matching visual point.

Persistent parent; SDK exists only inside a read-only child, never at prompts.
The optional h console sends only explicit operator hand targets. No arm moves.
"""
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import zipfile

ROOT=Path(__file__).resolve().parents[1]
MANIFEST=ROOT/'deployment/new-grasp-reference-v1-files.sha256'
PRE=[35000,7500,9480,30000,65535,65535]


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    from known_cylinder_records import dump,archive_folder
    folder=ROOT/'captures'/datetime.now().strftime('new_grasp_reference_%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True,exist_ok=False)
    report=dict(scope='new_grasp_seated_wrist_then_same_shell_visual_binding',
        outcome='recording',arm_motion_writes=0,arm_stop_writes=0,configuration_writes=0,
        parent_SDK_loaded=False,execution_ready=False,exceptions=[],read_attempts=[],
        hand_console_can_send_only_explicit_targets=True,physical_grip_success_source='operator_followup')
    current=None;child=None

    def save():dump(folder/'result.json',report)

    def invalidate_reference():
        nonlocal current
        current=None;old_hash=report.get('task_anchor_sha256')
        local=folder/'new_task_anchor.json'
        if local.exists():local.replace(folder/(datetime.now().strftime('superseded_%H%M%S_%f_')+local.name))
        pointer=ROOT/'calibration/black_shell_new_grasp_task_anchor.json'
        if old_hash and pointer.exists() and sha(pointer)==old_hash:
            pointer.replace(pointer.with_name(datetime.now().strftime('superseded_%Y%m%d_%H%M%S_%f_')+pointer.name))
        report.pop('task_anchor',None);report.pop('task_anchor_sha256',None)
        report['outcome']='needs_new_seated_wrist_reference';save()

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

    def read_hand():
        from manual_hand_bus_v2 import ManualHandBus
        bus=ManualHandBus(lambda row:append_wire(row))
        try:return bus.connect_read()
        finally:bus.close()

    def append_wire(row):
        with (folder/'hand_wire.jsonl').open('a',encoding='utf8') as stream:
            stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')

    def check_hand(row):
        if (row.get('ok') is not True or row.get('snapshot_stale') is not False
                or row.get('needs_reconnect') or not row.get('snapshot')):
            raise ValueError('未取得新的完整手部读数，记录保留，可r重读。')
        hand=row['snapshot']
        if len(hand.get('targets',[]))!=6 or any(abs(a-b)>2 for a,b in zip(hand['targets'],PRE)):
            raise ValueError('预抓目标尚非35000/7500/9480/30000/65535/65535；h进入手部工具，pre设置后q返回。')
        if any(v not in (0,1,2,5) for v in hand['status']):
            raise ValueError('当前有真实保护，仍保留界面。')
        # No actual-position arrival/settling requirement: status5 is normal contact.
        return hand

    def record_wrist():
        nonlocal current
        invalidate_reference()
        attempt=folder/('arm_reference_%03d.json'%(len(report['read_attempts'])+1))
        item=dict(timestamp_utc=datetime.now(timezone.utc).isoformat(),worker_file=attempt.name,
            operator_confirms_seated_shell_pregrasp_before_lift_not_post_lift=True)
        report['read_attempts'].append(item);save()
        before=read_hand();item['hand_before']=before;save();check_hand(before)
        code=run_child([sys.executable,str(ROOT/'deployment/new_grasp_reference_worker_v1.py'),
            '--output',str(attempt)])
        if not attempt.exists():raise ValueError('臂位子进程没有结果文件，可r重读；没有发送臂动作。')
        arm=json.loads(attempt.read_bytes());item['worker_exit_code']=code
        item['arm_result_sha256']=sha(attempt);save()
        if code or not arm.get('reference_qualified') or not arm.get('SDK_cleanup_finished'):
            raise ValueError('臂位读取未合格，详细原值已保存；保持界面，可r重读。')
        after=read_hand();item['hand_after']=after;save();check_hand(after)
        if any(abs(a-b)>2 for a,b in zip(before['snapshot']['targets'],after['snapshot']['targets'])):
            raise ValueError('读取期间手目标改变，本次不作参考，可r重读。')
        current=dict(arm=arm,hand=after,arm_file=attempt,arm_sha256=sha(attempt))
        item['new_reference_recorded']=True;save()
        print('新抓取前腕姿已记录，SDK已退出。空壳/头/板保持不动；手若挡视野，可只移开空手，再v采图。',flush=True)

    def capture_visual():
        if current is None:
            print('先把手摆回成功抓取前、空壳落底的抓取前位置，r记录腕姿。',flush=True);return
        before=set((ROOT/'captures').glob('new_grasp_reference_vision_*.zip'))
        command=[sys.executable,str(ROOT/'deployment/run_new_grasp_reference_vision_v1.py'),
            '--position-session',str(ROOT/'calibration/position_rgbd_20261020_001259_216986'),
            '--anchor',str(ROOT/'deployment/visual_reference_201634/task_anchor.json'),
            '--confirm-fixed-camera','--confirm-measured-shell-only','--confirm-unchanged-setup',
            '--confirm-upright-equal-height-recesses','--finish-after-first-reference',
            '--inference-python','/usr/bin/python3','--device','auto']
        report['vision_command']=command;save()
        code=run_child(command)
        created=set((ROOT/'captures').glob('new_grasp_reference_vision_*.zip'))-before
        if code or len(created)!=1:raise ValueError('视觉参考尚未合格，当前腕姿记录保留；空壳未动时可v再采。')
        source=created.pop();copy_path=folder/('vision_reference_%03d.zip'%(len(report.get('vision_sources',[]))+1))
        copy_path.write_bytes(source.read_bytes())
        with zipfile.ZipFile(copy_path) as archive:
            if archive.testzip() is not None:raise ValueError('视觉结果ZIP不完整。')
            member_names=archive.namelist()
            manifests=[n for n in member_names if n.endswith('/source_hashes.json')]
            if len(manifests)!=1:raise ValueError('视觉结果缺少唯一文件SHA记录。')
            prefix=manifests[0][:-len('source_hashes.json')]
            hashes=json.loads(archive.read(manifests[0]))
            for name,digest in hashes.items():
                if '/' in name or '\\' in name or hashlib.sha256(archive.read(prefix+name)).hexdigest()!=digest:
                    raise ValueError('视觉源文件SHA不符：'+name)
            point=json.loads(archive.read(prefix+'latest_reference_point.json'))
        report.setdefault('vision_sources',[]).append(dict(file=copy_path.name,sha256=sha(copy_path)))
        recipe=json.loads((ROOT/'deployment/black_shell_new_grasp_recipe_20261023.json').read_bytes())
        from new_grasp_task_anchor_v1 import make_anchor
        anchor=make_anchor(current['arm'],current['hand'],point,recipe,
            dict(arm_reference_file=current['arm_file'].name,arm_reference_sha256=current['arm_sha256'],
                vision_archive_file=copy_path.name,vision_archive_sha256=sha(copy_path),
                recipe_sha256=sha(ROOT/'deployment/black_shell_new_grasp_recipe_20261023.json')))
        dump(folder/'new_task_anchor.json',anchor)
        destination=ROOT/'calibration/black_shell_new_grasp_task_anchor.json'
        destination.parent.mkdir(parents=True,exist_ok=True);dump(destination,anchor)
        report.update(outcome='new_wrist_and_same_place_visual_reference_recorded',task_anchor=anchor,
            task_anchor_sha256=sha(folder/'new_task_anchor.json'));save()
        print('新腕姿与同位置视觉参考已绑定。旧腕姿和旧2.8+4.5mm偏移未使用；输入q保存统一ZIP。',flush=True)

    try:
        checked=0
        for line in MANIFEST.read_text(encoding='ascii').splitlines():
            digest,name=line.split('  ',1);p=(ROOT/name).resolve()
            if ROOT not in p.parents or sha(p)!=digest:raise ValueError('交付文件不符：'+name)
            checked+=1
        if not checked:raise ValueError('交付清单为空。')
        report['checked_files']=checked;save()
        print('新抓取基准：不自动移臂，不重做标定或路径。先让空壳完全落底，再把空手摆到这次成功抓取前的抓取前腕姿/位置。',flush=True)
        print('r表示已退出示教、停稳撤手，确认是抓取前位置（不是提起后的当前位置）；读取期间保持不动。',flush=True)
        while True:
            try:
                print('h调手(pre预抓、grip拇→食→中、q返回) | r记录当前新腕姿 | v采同位置空壳视觉 | q保存退出',flush=True)
                answer=input('> ').strip().lower()
                if answer=='q':break
                if answer=='h':
                    invalidate_reference()
                    run_child([sys.executable,str(ROOT/'deployment/run_hand_thumb_tuner_v2.py'),'--execute'])
                    print('调手后请停稳，再r记录当前抓取前位置。',flush=True)
                elif answer=='r':record_wrist()
                elif answer=='v':capture_visual()
                else:print('输入h、r、v或q；界面保留。',flush=True)
            except KeyboardInterrupt:print('本次已中断，保留界面；未自动重试。',flush=True)
            except EOFError:break
            except Exception as exc:
                report['exceptions'].append(dict(error=repr(exc),timestamp_utc=datetime.now(timezone.utc).isoformat()))
                save();print(str(exc),flush=True)
    except Exception as exc:
        report['exceptions'].append(dict(error=repr(exc)));report['outcome']='delivery_not_available_no_device_commands'
        print(str(exc),flush=True)
    finally:
        report['execution_ready']=False;save()
        archive=archive_folder(folder)
        print('SAVED:',archive,'SHA256:',sha(archive),flush=True)


if __name__=='__main__':main()
