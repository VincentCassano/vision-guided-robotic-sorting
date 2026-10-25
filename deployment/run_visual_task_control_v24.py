#!/usr/bin/env python3
"""Local web control and continuous D435 video; motion requires START."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import webbrowser
import zipfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/battery_grasp/src'))


def verify_files():
    for name in ('visual-pick-sort-v23-files.sha256','visual-task-control-v24-files.sha256'):
        manifest=ROOT/'deployment'/name
        for line in manifest.read_text(encoding='ascii').splitlines():
            digest,relative=line.split('  ',1)
            path=(ROOT/relative).resolve()
            if ROOT not in path.parents or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
                raise ValueError('交付文件校验不符：'+relative)


def open_browser(url,width,height):
    for binary in ('chromium','chromium-browser','google-chrome','google-chrome-stable'):
        path=shutil.which(binary)
        if path:
            subprocess.Popen([path,'--app='+url,'--window-size=%d,%d'%(width,height),
                              '--window-position=60,35'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            return
    # Other browsers retain their own window preferences. Never request fullscreen.
    webbrowser.open(url,new=1)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=18081)
    parser.add_argument('--inference-python',default='/usr/bin/python3')
    parser.add_argument('--device',default='auto')
    parser.add_argument('--no-browser',action='store_true')
    parser.add_argument('--window-width',type=int,default=1440)
    parser.add_argument('--window-height',type=int,default=940)
    args=parser.parse_args()
    verify_files()
    import fcntl
    lock_path=ROOT/'calibration/visual_task_control_v24.lock'
    lock_path.parent.mkdir(parents=True,exist_ok=True)
    lock=lock_path.open('a+')
    try:
        fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        parser.exit(1,'网页控制服务已经运行，请打开 http://127.0.0.1:%d\n'%args.port)
    from visual_live_camera_v24 import LiveCamera
    from visual_task_control_v24 import RealTaskControl,create_server,atomic_json,STARTUP_MOTTO
    print(STARTUP_MOTTO,flush=True)
    session=ROOT/'captures'/datetime.now().strftime('real_task_control_v24_%Y%m%d_%H%M%S_%f')
    session.mkdir(parents=True)
    camera=LiveCamera(args.inference_python,args.device)
    endpoint='http://127.0.0.1:%d'%args.port
    control=RealTaskControl(endpoint,camera,session)
    server=create_server(args.host,args.port,control,camera)
    control.endpoint=endpoint='http://127.0.0.1:%d'%server.server_port
    atomic_json(session/'startup.json',dict(endpoint=endpoint,local_sdk_loaded=False,
        camera_serial='344322075008',camera_stream='RGB 1280x720 BGR8 30fps',
        preview='continuous_MJPEG_display_only',measurement='lossless_same_frame_BGR_and_YOLO',
        baseline='fix11_console1_lift40',motion_worker_modified=False,
        post_drop_raise_mm=dict(red=40,white=40,black=80),
        browser_window=dict(width=args.window_width,height=args.window_height,fullscreen=False),
        physical_field_acceptance=False))
    thread=threading.Thread(target=server.serve_forever,daemon=True)
    thread.start()
    try:
        camera.start()
        control.boot()
        print('网页真机控制：'+endpoint,flush=True)
        print('相机和YOLO持续运行；点击“自动分拣/单件”才执行原成功版动作。',flush=True)
        print('Ctrl+C请求停止并等待原动作进程收尾；日志：'+str(session),flush=True)
        if not args.no_browser:
            open_browser(endpoint,args.window_width,args.window_height)
        threading.Event().wait()
    except KeyboardInterrupt:
        print('请求停止，等待原动作进程收尾；软件停止不能代替硬件急停。',flush=True)
    finally:
        try:
            control.close()
            atomic_json(session/'final_status.json',control.status())
        finally:
            try:
                camera.close()
            finally:
                server.shutdown()
                server.server_close()
                thread.join(timeout=2)
                lock.close()
                output=session.with_suffix('.zip')
                with zipfile.ZipFile(output,'x',zipfile.ZIP_DEFLATED) as archive:
                    for path in sorted(session.rglob('*')):
                        if path.is_file():archive.write(path,session.name+'/'+path.relative_to(session).as_posix())
                    pointer=json.loads((session/'runner_pointer.json').read_bytes()) if (session/'runner_pointer.json').exists() else {}
                    if pointer.get('result_file'):
                        robot_archive=Path(pointer['result_file']).parent.with_suffix('.zip')
                        if robot_archive.is_file():
                            archive.write(robot_archive,session.name+'/robot_session.zip',compress_type=zipfile.ZIP_STORED)
                print('SAVED:',output,'SHA256:',hashlib.sha256(output.read_bytes()).hexdigest(),flush=True)


if __name__=='__main__':
    main()
