"""Read-only hand TCP with a narrowly scoped arm-completion event lane.

Official protocol: https://develop.realman-robotics.com/robot/json/motionConfig/
An event is telemetry only, never a register response or endpoint permission.
All other unsolicited/duplicate/wrong-command frames still fail closed.
"""
import json
import select
import time
from tuned_grip_bin_hand import HeldReadTransport
from school_battery_transport import MULTI,SINGLE,WRITE,TransportError


class MotionEventReadTransport(HeldReadTransport):
    def __init__(self,sock,log,authority,event_context):
        super().__init__(sock,log,authority)
        self.event_context=event_context

    def frame(self,deadline):
        while b'\n' not in self.buffer:
            left=deadline-time.monotonic()
            if left<=0:raise TransportError('Frame deadline exceeded')
            self.sock.settimeout(left);self.receive()
        line,self.buffer=self.buffer.split(b'\n',1)
        if not line.endswith(b'\r'):raise TransportError('Frame lacks CRLF')
        return json.loads(line.decode('utf8'))

    def arm_event(self,row):
        if not isinstance(row,dict) or row.get('state')!='current_trajectory_state':return False
        context=self.event_context()
        if (set(row)!={'state','trajectory_state','device','trajectory_connect'}
                or type(row.get('device')) is not int or row['device']!=0
                or type(row.get('trajectory_connect')) is not int or row['trajectory_connect']!=0
                or type(row.get('trajectory_state')) is not bool
                or not isinstance(context,dict) or context.get('acknowledged') is not True
                or context.get('method') not in ('Movej_P_Cmd','Movel_Cmd')
                or context.get('event_seen') is not False):
            raise TransportError('Arm notification outside this live acknowledged segment or duplicate/invalid event')
        context['event_seen']=True
        self.log(dict(event='arm_motion_notification',sequence=self.sequence,monotonic=time.monotonic(),
            notification=row,segment=context.copy(),used_as_hand_response=False,
            endpoint_or_retention_qualified=False))
        if row['trajectory_state'] is not True:raise TransportError('Controller reported trajectory failure')
        return True

    def exchange(self,message):
        # This dedicated transport has no write lane, including normal opening.
        try:
            self._validate(message)
            if message.get('command') not in (MULTI,SINGLE) or self.broken:
                raise TransportError('Moving hand lane permits healthy read requests only')
        except BaseException as exc:
            self.latch_fault(repr(exc));raise
        self.sequence+=1;started=time.monotonic()
        try:
            quiet=self.pre_send_quiet_seconds(message);quiet_end=time.monotonic()+quiet
            # Preserve the quiet window. A complete known event may occur before
            # send; ordinary hand responses here remain unsolicited and fatal.
            while True:
                if self.buffer:
                    if not self.arm_event(self.frame(started+3)):
                        raise TransportError('Unsolicited or late response before send')
                    continue
                left=quiet_end-time.monotonic()
                if left<=0:break
                if not select.select([self.sock],[],[],left)[0]:break
                self.receive()
            payload=(json.dumps(message,separators=(',',':'))+'\r\n').encode('ascii')
            self.log(dict(event='tx',sequence=self.sequence,monotonic=time.monotonic(),request=message,
                pre_send_quiet_seconds=quiet,hex=payload.hex()))
            self.sock.sendall(payload);deadline=time.monotonic()+3
            while True:
                response=self.frame(deadline)
                if self.arm_event(response):continue
                break
            if not isinstance(response,dict) or response.get('command')!=message['command']:
                raise TransportError('Response command mismatch')
            if response.get('read_state') is False:raise TransportError('Read rejected')
            data=response.get('data')
            if message['command']==SINGLE:
                if type(data) is not int or not -32768<=data<=65535:raise TransportError('Invalid single register')
                value=data&65535
            else:
                if (not isinstance(data,list) or len(data)!=2*message['num']
                        or any(type(v) is not int or not 0<=v<=255 for v in data)):
                    raise TransportError('Wrong response byte count/type')
                value=[data[i]*256+data[i+1] for i in range(0,len(data),2)]
            # A coalesced event after a response is routed separately. A second
            # hand response, unexpected event or partial-frame timeout aborts.
            while self.buffer:
                if not self.arm_event(self.frame(deadline)):
                    raise TransportError('Duplicate/extra response in same receive')
            self.log(dict(event='response',sequence=self.sequence,monotonic=time.monotonic(),
                elapsed_s=time.monotonic()-started,response=response,value=value))
            self.previous=(message['command'],message.get('num'))
            return value
        except BaseException as exc:
            self.broken=True
            self.log(dict(event='transport_error',sequence=self.sequence,error=repr(exc),
                pending_request=message,buffer_hex=self.buffer.hex()))
            self.latch_fault(repr(exc));raise
