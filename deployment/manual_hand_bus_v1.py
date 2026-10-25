"""Manual six-channel hand targets through TCP only; no SDK or arm commands.

Every public operation returns feedback to its caller. Hardware protection,
framing and configuration failures inhibit a write, never terminate the UI.
Only an explicit caller command can attempt one1135..1140 position write.
"""
import copy
import json
from pathlib import Path
import select
import socket
import time

from school_battery_transport import MULTI, SINGLE, WRITE, TransportError
from supported_contact_transport import SupportedContactTransport, PhaseAuthority
from school_device_identity import identity_fields, same_live_identity, configuration_matches


ROOT = Path(__file__).resolve().parents[1]
CONFIG_BLOCKS = (
    ('identity',1000,8),('calibration_end_absolute',1020,6),
    ('calibration_start_absolute',1030,6),('thumb_presets_absolute',1040,3),
    ('pid_p',1045,6),('pid_i',1055,6),('pid_d',1065,6),('pid_g',1075,6),
    ('current_limit_ma',1095,6),('stall_speed',1185,6),
    ('stall_current_ma',1195,6),('stall_stop_ms',1205,6),('stall_retry_ms',1215,6))


class _ManualTransport(SupportedContactTransport):
    """An exact one-use position permit; all other traffic remains read-only."""
    def __init__(self,sock,log):
        super().__init__(sock,log,PhaseAuthority())
        self.identity_raw=None
        self.permit=None
        self.permit_serial=None
        self.last_write_tx_started=False
        self.last_write_acknowledged=False

    def grant(self,channel,value,serial):
        if self.broken or self.terminal_latched or self.permit is not None:
            raise TransportError('Current hand socket has no new target capability')
        self.permit=(1135+channel,(value,))
        self.permit_serial=serial
        self.last_write_tx_started=False;self.last_write_acknowledged=False
        self.motion_enabled=True

    def _validate(self,message):
        if not isinstance(message,dict) or message.get('command')!=WRITE:
            return super()._validate(message)
        if self.permit is None:
            raise TransportError('No explicit single-channel target permit')
        address,values=self.permit
        expected=dict(command=WRITE,port=1,device=2,address=address,num=1,
            data=[values[0]>>8,values[0]&255])
        if (message!=expected or any(type(message.get(k)) is not int for k in ('port','device','address','num'))
                or any(type(v) is not int for v in message.get('data',[]))
                or not 1135<=address<=1140 or self.broken or self.terminal_latched
                or not self.motion_enabled):
            raise TransportError('Only this explicit single-channel position target is permitted')
        self.permit=None
        return address,values

    def _frame(self,deadline):
        while b'\n' not in self.buffer:
            remaining=deadline-time.monotonic()
            if remaining<=0:raise TransportError('Hand frame deadline exceeded')
            self.sock.settimeout(remaining);self.receive()
        line,self.buffer=self.buffer.split(b'\n',1)
        if not line.endswith(b'\r'):raise TransportError('Hand frame lacks CRLF')
        return json.loads(line[:-1].decode('utf8'))

    def _arm_telemetry(self,row):
        if not isinstance(row,dict) or row.get('state')!='current_trajectory_state':return False
        if (set(row)!={'state','trajectory_state','device','trajectory_connect'}
                or type(row.get('device')) is not int or row['device']!=0
                or type(row.get('trajectory_state')) is not bool
                or type(row.get('trajectory_connect')) is not int or row['trajectory_connect'] not in (0,1)):
            raise TransportError('Unrecognized asynchronous trajectory notification')
        self.log(dict(event='external_arm_telemetry_ignored_by_manual_hand_tool',
            monotonic=time.monotonic(),notification=row,used_as_hand_response=False,
            arm_control_or_stop_claimed=False))
        return True

    def exchange(self,message):
        signature=None
        try:
            signature=self._validate(message)
            if self.broken:raise TransportError('Hand socket is faulted; explicit reconnect required')
            if signature is not None:self._attempted_writes.add(signature)
            self.sequence+=1;started=time.monotonic();quiet_end=started+.300
            while True:
                if time.monotonic()-started>3:raise TransportError('Unsolicited-frame drain deadline exceeded')
                if self.buffer:
                    if not self._arm_telemetry(self._frame(started+3)):
                        raise TransportError('Unexpected or late hand response before send')
                    continue
                left=quiet_end-time.monotonic()
                if left<=0 or not select.select([self.sock],[],[],left)[0]:break
                self.receive()
            payload=(json.dumps(message,separators=(',',':'))+'\r\n').encode('ascii')
            self.log(dict(event='tx',sequence=self.sequence,monotonic=time.monotonic(),
                request=message,hex=payload.hex(),operator_command_serial=self.permit_serial,
                pre_send_quiet_seconds=.300))
            if signature is not None:self.last_write_tx_started=True
            self.sock.sendall(payload);deadline=time.monotonic()+3
            while True:
                response=self._frame(deadline)
                if not self._arm_telemetry(response):break
            if not isinstance(response,dict) or response.get('command')!=message['command']:
                raise TransportError('Hand response command mismatch')
            if message['command']==WRITE:
                if response.get('write_state') is not True:
                    raise TransportError('Target write rejected or ACK uncertain')
                value=None;self.last_write_acknowledged=True
            else:
                if response.get('read_state') is False:raise TransportError('Hand read rejected')
                data=response.get('data')
                if message['command']==SINGLE:
                    if type(data) is not int or not -32768<=data<=65535:
                        raise TransportError('Invalid single-register response')
                    value=data&65535
                else:
                    if (not isinstance(data,list) or len(data)!=2*message['num']
                            or any(type(v) is not int or not 0<=v<=255 for v in data)):
                        raise TransportError('Invalid register response count or byte type')
                    value=[data[i]*256+data[i+1] for i in range(0,len(data),2)]
            while self.buffer:
                if not self._arm_telemetry(self._frame(deadline)):
                    raise TransportError('Duplicate or extra hand response')
            if message['command']==MULTI and message.get('address')==1000:
                identity_fields(value)
                if self.identity_raw is not None and not same_live_identity(value,self.identity_raw):
                    raise TransportError('New hand diagnostic or identity event; reconnect to reread')
                self.identity_raw=value[:]
            self.log(dict(event='response',sequence=self.sequence,monotonic=time.monotonic(),
                elapsed_s=time.monotonic()-started,response=response,value=value,
                operator_command_serial=self.permit_serial))
            self.previous=(message['command'],message.get('num'))
            if signature is not None:
                self._phase_authority._acknowledged_writes.add(signature)
                self.motion_enabled=False
            return value
        except Exception as exc:
            self.broken=True;self.motion_enabled=False
            self.log(dict(event='manual_bus_transport_fault',sequence=self.sequence,
                monotonic=time.monotonic(),error=repr(exc),request=message,
                write_tx_started=self.last_write_tx_started,
                write_acknowledged=self.last_write_acknowledged,buffer_hex=self.buffer.hex()))
            self.latch_fault(str(exc))
            raise


class ManualHandBus:
    """Persistent manual UI backend. No expected hardware failure escapes here."""
    def __init__(self,log):
        self._sink=log;self._io=None;self._serial=0;self._generation=0
        self._profile=None;self._configuration=None;self._configuration_matched=False
        self._reconnect_reason=None;self._logging_error=None
        self.last_snapshot=None;self._snapshot_stale=True

    def _log(self,row):
        try:self._sink(dict(row,connection_generation=self._generation))
        except Exception as exc:self._logging_error=repr(exc)

    def _result(self,ok,notice,**fields):
        result=dict(ok=bool(ok),notice=str(notice),snapshot=copy.deepcopy(self.last_snapshot),
            snapshot_stale=self._snapshot_stale,connected=self._io is not None,
            needs_reconnect=self._io is None or self._reconnect_reason is not None,
            configuration_matched=self._configuration_matched,
            blockers=copy.deepcopy(self.last_snapshot.get('blockers',[]) if self.last_snapshot else []),
            attempted=False,acknowledged=False,physical_stop_verified=False,holding_verified=False)
        result.update(fields);return result

    def close(self):
        io,self._io=self._io,None
        if io is not None:
            io.disarm_motion()
            try:io.sock.close()
            except Exception as exc:self._log(dict(event='manual_socket_close_error',error=repr(exc)))
        self._snapshot_stale=True
        return self._result(True,'连接已关闭；未写停止、复位或保护参数')

    def _failed(self,exc,attempted=False,acknowledged=False,**fields):
        self._reconnect_reason=str(exc)
        self._log(dict(event='manual_operation_unavailable',monotonic=time.monotonic(),
            error=repr(exc),attempted=attempted,acknowledged=acknowledged,
            automatic_retry=False,physical_stop_verified=False))
        self.close()
        return self._result(False,'连接暂停：'+str(exc)+'；r重新读取，原指令不会自动重发',
            attempted=attempted,acknowledged=acknowledged,**fields)

    def connect_read(self):
        self.close();self._reconnect_reason=None;self._configuration_matched=False
        self._configuration=None;self.last_snapshot=None;self._generation+=1
        self._logging_error=None
        try:
            self._profile=json.loads((ROOT/'deployment/battery_hand_verified_profile.json').read_bytes())
            if (self._profile.get('endpoint')!='169.254.128.18:8080'
                    or self._profile.get('port')!=1 or self._profile.get('device')!=2):
                return self._result(False,'本机手配置来源不符，仅保留界面；请核对部署文件')
            sock=socket.create_connection(('169.254.128.18',8080),timeout=3)
            self._io=_ManualTransport(sock,self._log)
            actual={name:self._io.read(address,count) for name,address,count in CONFIG_BLOCKS}
            self._configuration=actual
            self._configuration_matched=configuration_matches(actual,self._profile['observed_configuration'])
            self._log(dict(event='manual_full_hand_configuration',monotonic=time.monotonic(),
                actual=actual,matched=self._configuration_matched,SDK_loaded=False,
                arm_state_queried=False,configuration_writes=0))
            return self.refresh()
        except Exception as exc:return self._failed(exc)

    def _state(self):
        io=self._io;began=time.monotonic()
        identity=io.read(1000,8)
        limits=io.read(1095,6);speeds=io.read(1125,6);forces=io.read(1115,5)
        targets=io.read(1135,6);positions=io.read(1145,6)
        currents=io.read(1105,5)+[io.single(1110)];status=io.read(1085,6)
        blockers=[]
        if not self._configuration_matched:blockers.append('完整手配置与本机profile不匹配，当前只读')
        if limits!=self._profile.get('current_limit_ma'):blockers.append('本机硬件限流值改变，当前只读')
        if speeds!=[65535]*6:blockers.append('本机速度配置改变，当前只读')
        if any(forces):blockers.append('当前有力控目标，不能混用位置调节')
        if any(s not in range(6) for s in status):blockers.append('状态码无效，当前只读')
        if any(limit<=0 or current>limit for current,limit in zip(currents,limits)):
            blockers.append('实测电流超过当前硬件限流，暂停下发')
        if self._logging_error:blockers.append('原始记录写入失败：'+self._logging_error)
        row=dict(identity=identity,current_limit_ma=limits,speeds=speeds,force_targets=forces,
            targets=targets,positions=positions,current=currents,status=status,
            protected_channels=[ch for ch,s in enumerate(status) if s in (3,4)],
            status5_channels=[ch for ch,s in enumerate(status) if s==5],
            blockers=blockers,started_monotonic=began,finished_monotonic=time.monotonic(),
            reads_are_sequential=True,physical_stop_verified=False,holding_verified=False,
            actual_target_differences=[p-t for p,t in zip(positions,targets)])
        self.last_snapshot=row;self._snapshot_stale=False
        self._log(dict(event='manual_hand_snapshot',snapshot=row))
        return row

    def refresh(self):
        if self._io is None:return self._result(False,'尚未连接；r重新读取')
        if self._reconnect_reason is not None:return self._result(False,'需先r重新读取；未发指令')
        try:
            row=self._state()
            notice='已读取；状态5或未到位可继续调，保护3/4通道只可退让'
            if row['blockers']:notice='；'.join(row['blockers'])
            return self._result(not row['blockers'],notice)
        except Exception as exc:return self._failed(exc)

    def set_target(self,channel,absolute=None,delta=None):
        self._serial+=1;serial=self._serial
        if (type(channel) is not int or channel not in range(6)
                or (absolute is None)==(delta is None)
                or type(absolute if absolute is not None else delta) is not int):
            return self._result(False,'请输入一个通道和整数目标或相对步幅；未发指令')
        if self._io is None or self._reconnect_reason is not None:
            return self._result(False,'当前连接不可写；r仅重新读取，不会重发原指令')
        attempted=False;acknowledged=False;command=None
        try:
            before=self._state()
            requested=absolute if absolute is not None else before['positions'][channel]+delta
            value=max(0,min(65535,requested))
            command=dict(serial=serial,channel=channel,address=1135+channel,
                input_mode='absolute' if absolute is not None else 'delta_from_fresh_actual',
                requested=absolute if absolute is not None else delta,requested_absolute=requested,
                actual_basis=before['positions'][channel],applied=value,clamped=value!=requested,
                uncommanded_channels=[ch for ch in range(6) if ch!=channel],
                one_attempt=True,automatic_retry=False)
            self._log(dict(event='explicit_manual_target_request',command=command,before=before))
            if self._logging_error:
                return self._result(False,'原始记录写入失败；保留界面，未发指令',command=command)
            if before['blockers']:
                return self._result(False,'；'.join(before['blockers'])+'；未发指令',command=command)
            protected=before['status'][channel] in (3,4)
            if protected and value>min(before['targets'][channel],before['positions'][channel]):
                return self._result(False,'该路保护3/4只允许减小目标退让；其他健康通道仍可调',command=command)
            if value==before['targets'][channel]:
                return self._result(True,'目标未变，不重发；可输入新值继续调',command=command,
                    actual_target_arrival_qualified=abs(before['positions'][channel]-value)<=150)
            self._io.grant(channel,value,serial)
            attempted=True
            self._io.write_group(1135+channel,[value]);acknowledged=True
            after=self._state()
            changed_others=[ch for ch in range(6) if ch!=channel
                and abs(after['targets'][ch]-before['targets'][ch])>2]
            if abs(after['targets'][channel]-value)>2 or changed_others:
                self._reconnect_reason='目标回读与本次ACK不一致'
                return self._result(False,'目标回读不一致，保留界面；r新读后再手动决定',
                    command=command,attempted=True,acknowledged=True,
                    externally_changed_target_channels=changed_others)
            reached=abs(after['positions'][channel]-after['targets'][channel])<=150
            notice='已下发并收到ACK；'+('当前接近目标' if reached else '当前未到位，可继续调')
            if after['status'][channel]==5:notice+='；状态5仅显示，不退出'
            if after['status'][channel] in (3,4):notice+='；该路保护停止，仍可退让/调其他指'
            if command['clamped']:notice+='；已按0..65535限幅'
            if after['blockers']:notice+='；'+'；'.join(after['blockers'])
            return self._result(not after['blockers'],notice,command=command,attempted=True,
                acknowledged=True,actual_target_arrival_qualified=reached,
                arrival_or_settling_waited=False)
        except Exception as exc:
            if attempted and self._io is not None:
                acknowledged=acknowledged or self._io.last_write_acknowledged
            return self._failed(exc,attempted=attempted,acknowledged=acknowledged,command=command)
