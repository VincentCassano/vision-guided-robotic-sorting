"""Exact grouped hand-position capabilities on the proven v2 wire transport.

Only source PRE6, source GRIP3, and five-flex zero opening are writable.
The protocol/pacing, live reads and failure behavior remain inherited.
"""
import json
import socket
import time

from manual_hand_bus_v2 import ManualHandBus,_ManualTransport,CONFIG_BLOCKS,ROOT
from school_battery_transport import WRITE,TransportError
from school_device_identity import configuration_matches


def counts(value,n):
    if (not isinstance(value,(list,tuple)) or len(value)!=n
            or any(type(v) is not int or not 0<=v<=65535 for v in value)):
        raise ValueError('Complete unsigned hand position vector required')
    return list(value)


class GroupTransport(_ManualTransport):
    def __init__(self,sock,log,prepare,grip):
        super().__init__(sock,log)
        self.groups={
            'prepare_after_descent':(1135,tuple(counts(prepare,6))),
            'grip_together':(1135,tuple(counts(grip,6)[:3])),
            'empty_travel_open':(1135,(0,0,0,0,0)),
            'normal_bin_open':(1135,(0,0,0,0,0))}
        self.group_label=None

    def grant_group(self,label,serial):
        if label not in self.groups or self.broken or self.terminal_latched or self.permit is not None:
            raise TransportError('No healthy exact grouped-position permit')
        self.permit=self.groups[label];self.group_label=label
        self.permit_serial=serial;self.last_write_tx_started=False
        self.last_write_acknowledged=False;self.motion_enabled=True
        return self.permit

    def _validate(self,message):
        if not isinstance(message,dict) or message.get('command')!=WRITE:
            return super()._validate(message)
        if self.group_label is None or self.permit!=self.groups.get(self.group_label):
            raise TransportError('Only the exact current grouped-position permit is writable')
        address,values=self.permit
        expected=dict(command=WRITE,port=1,device=2,address=address,num=len(values),
            data=[byte for value in values for byte in (value>>8,value&255)])
        if (message!=expected or self.broken or self.terminal_latched or not self.motion_enabled
                or any(type(message.get(k)) is not int for k in ('port','device','address','num'))
                or any(type(v) is not int for v in message.get('data',[]))):
            raise TransportError('Grouped command differs from its source-qualified position targets')
        self.permit=None;self.group_label=None
        return address,values


class GroupHandBus(ManualHandBus):
    def __init__(self,log,prepare,grip):
        super().__init__(log)
        self.prepare=counts(prepare,6);self.grip=counts(grip,6)

    def connect_read(self):
        # Same connection/profile/working-read process as ManualHandBus v2;
        # only the transport's exact position permit is specialized.
        self.close();self._reconnect_reason=None;self._configuration_matched=False
        self._configuration=None;self.last_snapshot=None;self._generation+=1
        self._logging_error=None
        try:
            self._profile=json.loads((ROOT/'deployment/battery_hand_verified_profile.json').read_bytes())
            if (self._profile.get('endpoint')!='169.254.128.18:8080'
                    or self._profile.get('port')!=1 or self._profile.get('device')!=2):
                return self._result(False,'Hand profile identity differs')
            sock=socket.create_connection(('169.254.128.18',8080),timeout=3)
            self._io=GroupTransport(sock,self._log,self.prepare,self.grip)
            actual={name:self._io.read(address,count) for name,address,count in CONFIG_BLOCKS}
            self._configuration=actual
            self._configuration_matched=configuration_matches(actual,self._profile['observed_configuration'])
            self._log(dict(event='group_full_hand_configuration',monotonic=time.monotonic(),
                actual=actual,matched=self._configuration_matched,configuration_writes=0))
            return self.refresh()
        except Exception as exc:return self._failed(exc)
