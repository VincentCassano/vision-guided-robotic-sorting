"""Real v23 process adapter and local HTTP UI. Never imports the robot SDK."""
import base64
from datetime import datetime,timezone,timedelta
import json
from pathlib import Path
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

ROOT = Path(__file__).resolve().parents[1]
STARTUP_MOTTO = '世上无难事，只要肯登攀。'


def read_json(path, default=None):
    try:
        return json.loads(Path(path).read_bytes())
    except (OSError, ValueError, TypeError):
        return default


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name+'.new')
    temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    temporary.replace(path)


class RealTaskControl:
    def __init__(self, endpoint, camera, session):
        self.endpoint, self.camera, self.session = endpoint, camera, Path(session)
        self.lock = threading.RLock()
        self.process = self.log = None
        self.ready = self.busy = False
        self.closing = False
        self.mode = 'AUTO'
        self.phase = self.reason = ''
        self.report = {}
        self.pointer = self.session/'runner_pointer.json'
        self.stop_latch = self.session/'stop_requested.json'
        self.events = self.session/'control_events.jsonl'
        self.ui_logs=[]
        self._last_cycle_count=0
        self.last_refresh = 0.
        self.exit_code = None

    def boot(self):
        import os
        self._log('info',STARTUP_MOTTO)
        env = dict(os.environ, BATTERY_LIVE_ENDPOINT=self.endpoint,
            BATTERY_CONTROL_POINTER=str(self.pointer), BATTERY_CONTROL_STOP=str(self.stop_latch),
            PYTHONUNBUFFERED='1')
        self.log = (self.session/'runner_console.log').open('w',encoding='utf8')
        self.process = subprocess.Popen([sys.executable,str(ROOT/'deployment/run_visual_pick_sort_live_v24.py')],
            cwd=str(ROOT),stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
            text=True,bufsize=1,env=env)
        self.reader = threading.Thread(target=self._read,daemon=True)
        self.reader.start()

    def _event(self, command, result):
        with self.events.open('a',encoding='utf8') as stream:
            stream.write(json.dumps(dict(monotonic=time.monotonic(),command=command,result=result),ensure_ascii=False)+'\n')
        self._log('info' if result['http_status']<400 else 'warning',command+'：'+result['message'])

    def _log(self, level, message):
        row=dict(time=datetime.now(timezone(timedelta(hours=8))).strftime('%m-%d %H:%M:%S'),
            level=level,message=message)
        self.ui_logs.append(row)
        del self.ui_logs[:-80]
        with (self.session/'ui_logs.jsonl').open('a',encoding='utf8') as stream:
            stream.write(json.dumps(row,ensure_ascii=False)+'\n')

    def _read(self):
        try:
            for line in self.process.stdout:
                self.log.write(line)
                self.log.flush()
                with self.lock:
                    if line.strip()=='CONTROL_READY':
                        self.last_refresh=0
                        self._refresh()
                        self.ready = True
                        self.busy = False
                        self._log('info','任务控制已就绪，等待指令')
                    elif '当前：' in line:
                        phase=line.strip().split('当前：',1)[1]
                        if phase!=self.phase:self._log('info','当前：'+phase)
                        self.phase=phase
                    elif line.startswith(('RuntimeError(', 'ValueError(', '启动未完成：','任务暂停，')):
                        self.reason=line.strip()
                        self._log('warning',self.reason)
        finally:
            code=self.process.wait()
            with self.lock:
                self.ready = self.busy = False
                self.exit_code = code
            self.log.close()

    def _refresh(self):
        if time.monotonic()-self.last_refresh < .5:
            return
        self.last_refresh=time.monotonic()
        pointer=read_json(self.pointer,{})
        path=pointer.get('result_file')
        if path:
            path=Path(path).resolve()
            if ROOT/'captures' in path.parents:
                latest=read_json(path)
                if latest is not None:
                    self.report=latest
                    count=((latest.get('batches') or [{}])[-1]).get('command_cycles_completed',0)
                    if count>self._last_cycle_count:
                        self._log('info','本批动作周期完成：'+str(count)+'；实际入框未自动确认')
                    self._last_cycle_count=count

    def status(self):
        with self.lock:
            self._refresh()
            batch=(self.report.get('batches') or [{}])[-1]
            attempt=(self.report.get('attempts') or [{}])[-1]
            state=self.report.get('state','STARTING')
            if self.exit_code is not None:
                state='CLOSED' if self.exit_code==0 else 'ERROR'
            if self.busy and self.phase=='请求停止':
                state='STOPPING'
            phase=self.phase
            if state=='OBSERVING':phase='视觉识别与定位'
            elif state=='DONE':phase='本批结束'
            elif state=='VISION_PENDING':phase='视觉尚未确认清空'
            elif state=='IDLE' and not self.busy:phase='等待启动'
            reason=self.reason or (self.report.get('last_action_reason','') if state in ('PAUSED','STOPPED','ERROR','STARTUP_NOT_COMPLETED') else '')
            return dict(schema_version=24,simulation_only=False,backend='successful_v23_process',
                state=state,phase=phase,reason=reason,
                ready=self.ready and not self.closing,busy=self.busy,closing=self.closing,mode=self.mode,exit_code=self.exit_code,
                completed=batch.get('command_cycles_completed',0),
                targets_absent=batch.get('targets_absent_in_fresh_scan',0),
                actual_sorting_success=None,physical_stop_verified=False,
                locked_target=attempt.get('display_target') if state=='MOVING' and attempt.get('mode')=='visual_cycle' else None,
                target_color=attempt.get('target_color'),logical_bin=attempt.get('logical_bin'),
                wrist_profile=attempt.get('wrist_profile'),camera=self.camera.metadata(),
                logs=self.ui_logs[-40:],
                session=str(self.session),runner_result=read_json(self.pointer,{}).get('result_file'))

    def _send(self, letter):
        if self.process is None or self.process.poll() is not None:
            raise RuntimeError('任务进程已退出，请保留日志并重新启动控制服务')
        self.process.stdin.write(letter+'\n')
        self.process.stdin.flush()

    def _stop_files(self):
        pointer=read_json(self.pointer,{})
        for key in ('stop_file','abort_file'):
            value=pointer.get(key)
            if value:
                path=Path(value).resolve()
                if ROOT/'captures' in path.parents:
                    atomic_json(path,dict(operator_requested_stop=True,source='real_task_control_v24'))

    def command(self, command):
        command=str(command).upper()
        with self.lock:
            status=self.status()
            code=200
            message='已接收'
            try:
                if command=='STATUS':
                    return dict(status,http_status=200,accepted=True)
                if self.closing and command!='STOP':
                    raise ValueError('控制服务正在关闭，不再启动新任务')
                if command=='STOP':
                    if self.busy:
                        atomic_json(self.stop_latch,dict(operator_requested_stop=True))
                        self._stop_files()
                        self.phase='请求停止'
                        self.reason='已发送软件停止请求；以实际反馈和现场急停为准'
                        code=202
                    else:
                        message='当前没有执行中的任务'
                elif command in ('MODE_AUTO','MODE_MANUAL'):
                    if self.busy or not self.ready:
                        raise ValueError('任务执行中，不能切换模式')
                    self.mode=command[5:]
                elif command in ('START','START_AUTO','START_SINGLE'):
                    if not self.ready or self.busy:
                        raise ValueError('任务尚未就绪或仍在执行，不重复启动')
                    if status['state'] in ('PAUSED','STOPPED','INPUT_UNAVAILABLE','STARTUP_NOT_COMPLETED'):
                        raise ValueError('任务暂停；处理现场状态后可重置软件任务，不能自动重试')
                    cam=status['camera']
                    if not cam['measurement_ready'] or cam['frame_age_seconds'] is None or cam['frame_age_seconds']>1:
                        raise ValueError('实时相机/推理尚未就绪：'+str(cam.get('camera_error') or cam.get('inference_error') or '正在初始化'))
                    if command=='START_AUTO':self.mode='AUTO'
                    if command=='START_SINGLE':self.mode='MANUAL'
                    if self.stop_latch.exists():self.stop_latch.unlink()
                    self.ready=False
                    self.busy=True
                    self.phase='开始当前批次'
                    self.reason=''
                    self._send('a' if self.mode=='AUTO' else 'm')
                elif command=='RESET':
                    if self.busy or not self.ready:
                        raise ValueError('任务仍在执行或停止处理中，不重置设备')
                    self.ready=False
                    self.reason=''
                    self._send('r')
                    message='仅重置软件任务；不移动、松手或清设备错误'
                else:
                    code=400
                    message='未知指令'
            except ValueError as exc:
                code=409
                message=str(exc)
            except (OSError,RuntimeError) as exc:
                code=503
                message=str(exc)
                self.reason=message
            result=dict(self.status(),accepted=code<400,http_status=code,command=command,message=message)
            self._event(command,dict(http_status=code,message=message))
            return result

    def close(self):
        with self.lock:
            self.closing=True
        self.command('STOP')
        # Never terminate a live actuator process. Its original stop/cleanup
        # must return to the menu before sending the parent's exit command.
        while self.process is not None and self.process.poll() is None:
            with self.lock:
                if self.ready:
                    self._send('q')
                    self.ready=False
            try:
                self.process.wait(timeout=.5)
            except subprocess.TimeoutExpired:
                continue
        if self.process is not None:
            self.process.stdin.close()
            self.process.stdout.close()
            self.reader.join(timeout=2)


def create_server(host, port, control, camera):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):
            pass

        def reply(self,code,value,kind='application/json; charset=utf-8'):
            raw=json.dumps(value,ensure_ascii=False,allow_nan=False).encode('utf8') if not isinstance(value,bytes) else value
            self.send_response(code)
            self.send_header('Content-Type',kind)
            self.send_header('Content-Length',str(len(raw)))
            self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff')
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            parsed=urlsplit(self.path)
            try:
                if parsed.path=='/':
                    self.reply(200,(ROOT/'deployment/visual_task_dashboard_v24.html').read_bytes(),'text/html; charset=utf-8')
                elif parsed.path in ('/api/status','/status'):
                    self.reply(200,control.status())
                elif parsed.path=='/api/camera':
                    self.reply(200,camera.metadata())
                elif parsed.path=='/api/sample':
                    query=parse_qs(parsed.query)
                    meta,raw=camera.measurement(int(query.get('after',['-1'])[0]),float(query.get('since',['0'])[0]))
                    self.send_response(200)
                    self.send_header('Content-Type','application/octet-stream')
                    self.send_header('Content-Length',str(len(raw)))
                    self.send_header('Cache-Control','no-store')
                    self.send_header('X-Frame-Meta',base64.b64encode(json.dumps(meta,allow_nan=False).encode('utf8')).decode('ascii'))
                    self.end_headers()
                    self.wfile.write(raw)
                elif parsed.path=='/live.mjpg':
                    self.send_response(200)
                    self.send_header('Content-Type','multipart/x-mixed-replace; boundary=liveframe')
                    self.send_header('Cache-Control','no-store')
                    self.end_headers()
                    import cv2
                    while not camera.closing.is_set():
                        image=camera.render(control.status())
                        okay,encoded=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,82])
                        if okay:
                            raw=encoded.tobytes()
                            self.wfile.write(b'--liveframe\r\nContent-Type: image/jpeg\r\nContent-Length: '+
                                str(len(raw)).encode('ascii')+b'\r\n\r\n'+raw+b'\r\n')
                            self.wfile.flush()
                        time.sleep(.08)
                else:
                    self.reply(404,dict(error='Unknown route'))
            except (BrokenPipeError,ConnectionResetError):
                return
            except (TimeoutError,OSError,ValueError,RuntimeError) as exc:
                self.reply(503,dict(error=str(exc)))

        def do_POST(self):
            if urlsplit(self.path).path!='/api/command':
                self.reply(404,dict(error='Unknown route'))
                return
            origin=self.headers.get('Origin')
            if origin and urlsplit(origin).netloc!=self.headers.get('Host'):
                self.reply(403,dict(error='Same-origin commands only'))
                return
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0<length<=2048 or self.headers.get('Transfer-Encoding'):
                    raise ValueError('Expected a small JSON command')
                if not self.headers.get('Content-Type','').startswith('application/json'):
                    raise ValueError('Content-Type application/json required')
                body=json.loads(self.rfile.read(length))
                if set(body)!={'command'}:
                    raise ValueError('Only command is accepted')
                value=control.command(body['command'])
                self.reply(value['http_status'],value)
            except (ValueError,TypeError) as exc:
                self.reply(400,dict(error=str(exc)))

    server=ThreadingHTTPServer((host,port),Handler)
    server.daemon_threads=True
    return server
