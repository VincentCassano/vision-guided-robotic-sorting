"""Startup-only RTU restoration and full fresh grouped-hand connection.

No arm, power, hand-position, reset or fault-clear commands. Reuse the
previously field-used port1 RTU115200/timeout1 close/open transactions.
An old refused connection remains faulted; a replacement rereads everything.
"""
import copy
import io
import json
from hand_bus_startup import HandBusStartup,REQUEST as OPEN
from visual_bus_startup_v5_worker import probe_tool_mode
from visual_bus_reopen_v6_worker import close_once,CLOSE
from visual_pick_sort_v15_hand import GroupHandBus
from school_battery_transport import MULTI

EXPECTED_MODE=dict(command='get_tool_RS485_mode',tool_RS485_mode=1,
    baudrate=115200,modbus_timeout=1)


class RebootAwareGroupHandBus(GroupHandBus):
    def __init__(self,log,prepare,grip):
        self.read_request=None;self.reply_chunks=b''
        self.read_fault=None
        super().__init__(log,prepare,grip)

    def _log(self,row):
        if row.get('event')=='tx':
            self.read_request=copy.deepcopy(row.get('request'));self.reply_chunks=b''
        elif row.get('event')=='rx':
            self.reply_chunks+=bytes.fromhex(row['hex'])
        elif row.get('event')=='manual_bus_transport_fault':
            self.read_fault=dict(event=copy.deepcopy(row),request=copy.deepcopy(self.read_request),
                reply_hex=self.reply_chunks.hex())
        super()._log(row)

    def connect_startup(self,guard,permission,report,save,setup_log):
        setup=dict(attempts=[],mode_reads=[],bus_writes=[],reopen_attempted=False,
            power_writes=0,hand_register_writes=0,reset_or_clear_writes=0,
            old_failed_socket_reused=False,full_identity_configuration_feedback_required=True)
        report['hand_startup']=setup;save()
        def wire(row):
            if row.get('event')=='tx':
                request=row.get('request')
                if request not in (CLOSE,OPEN):
                    raise RuntimeError('No capability for other startup settings')
                setup['bus_writes'].append(copy.deepcopy(row))
                report['controller_configuration_writes']+=1
                report['hand_bus_configuration_writes']=report.get('hand_bus_configuration_writes',0)+1
                save()
            setup_log(row)
        def mode():
            with io.StringIO() as stream:
                row=probe_tool_mode(stream)
            setup['mode_reads'].append(row);setup_log(dict(event='tool_mode_get',**row));save()
            if row.get('recognized') is not True or row.get('close_error'):
                raise RuntimeError('Tool RS485 mode could not be read; see hand_startup mode reply')
            return row['response']
        def open_known():
            guard()
            if permission is not True:raise RuntimeError('Startup RTU setting permission missing')
            response=HandBusStartup(wire).configure_once()
            guard()
            if mode()!=EXPECTED_MODE:
                raise RuntimeError('RTU setup ACK without actual115200/timeout1 mode; no hand target')
            return response
        def read_all():
            self.read_fault=None
            row=super(RebootAwareGroupHandBus,self).connect_read()
            setup['attempts'].append(dict(result=copy.deepcopy(row),fault=copy.deepcopy(self.read_fault)))
            report['hand_connect_read']=copy.deepcopy(row);save()
            return row
        guard();before=mode();setup['mode_before']=copy.deepcopy(before)
        configured=False
        if before!=EXPECTED_MODE:
            setup['initial_open_response']=open_known();configured=True;save()
        row=read_all()
        if row.get('ok') is not True and self.first_identity_refused():
            # Only the known first identity read with an explicit refusal can
            # reach this setup. No actuator write has occurred. Never repeat a
            # hand target, reuse its latch, or treat a bad frame as a refusal.
            if configured or permission is not True:
                self.raise_unavailable(row)
            self.close();guard();setup['reopen_attempted']=True;save()
            try:
                setup['close_response']=close_once(wire);save()
                closed=mode()
                if closed.get('tool_RS485_mode')!=0:
                    raise RuntimeError('Closed tool serial mode0 not read back')
            finally:
                # Restore only the known original RTU mode once, including an
                # uncertain close. A failed restoration is never retried.
                if any(r.get('request')==CLOSE for r in setup['bus_writes']):
                    setup['reopen_response']=open_known();save()
            row=read_all()
        if row.get('ok') is not True:self.raise_unavailable(row)
        setup['outcome']='full_fresh_hand_reads_completed_not_object_absence_proof';save()
        return row

    def first_identity_refused(self):
        fault=self.read_fault or {}
        event=fault.get('event',{})
        if (event.get('sequence')!=1 or event.get('write_tx_started') is not False
                or event.get('write_acknowledged') is not False):
            return False
        if fault.get('request')!=dict(command=MULTI,port=1,address=1000,num=8,device=2):
            return False
        if fault.get('event',{}).get('error')!="TransportError('Hand read rejected')":
            return False
        try:
            raw=bytes.fromhex(fault.get('reply_hex',''))
            return raw.endswith(b'\r\n') and json.loads(raw)==dict(command=MULTI,read_state=False)
        except (ValueError,TypeError,UnicodeError):
            return False

    def raise_unavailable(self,row):
        fault=self.read_fault or {}
        request=fault.get('request') or {}
        reason=row.get('notice','')+'; blockers='+repr(row.get('blockers',[]))
        if request:reason+='; rejected_request='+repr(request)
        raise RuntimeError('Hand startup unavailable: '+reason+'; full raw result saved, no hand target sent')
