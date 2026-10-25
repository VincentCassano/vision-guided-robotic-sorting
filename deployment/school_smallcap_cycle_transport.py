"""Ordered position-only pilot; strict existing wire/fences, no force/reset writes."""
from supported_contact_transport import SupportedContactTransport,WritePhase
from school_battery_transport import WRITE

PREPARE=(1135,(8000,8000,8000,8000,65535,65535))
PLANS={'prepare_empty':(PREPARE,),
       'close':((1135,(15000, 30000, 28000, 20000, 65535, 65535)),),
       'release_to_prepare':(PREPARE,)}


class CycleTransport(SupportedContactTransport):
    def set_phase(self,phase,authority):
        if authority is not self._phase_authority or phase not in PLANS or self.terminal_latched or self.broken:
            self.latch_fault('Invalid cycle phase');raise RuntimeError('Cycle phase refused')
        order={'prepare_empty':0,'close':1,'release_to_prepare':2}
        prior=getattr(authority,'cycle_stage',-1)
        if order[phase]<=prior:raise RuntimeError('Repeated/out-of-order cycle phase')
        if phase=='close' and not getattr(authority,'prepare_feedback_pass',False):
            raise RuntimeError('Fresh PREPARE feedback required')
        if phase=='release_to_prepare' and prior!=1:raise RuntimeError('Release must follow controlled close')
        authority.cycle_stage=order[phase]
        self._cycle_phase=phase;self._phase_index=0
        self._phase={'prepare_empty':WritePhase.PREPARE_EMPTY,'close':WritePhase.CONTACT,
            'release_to_prepare':WritePhase.RELEASE}[phase]
        self.log(dict(event='position_cycle_phase',phase=phase,force_writes=False))

    def _validate(self,message):
        if message.get('command')!=WRITE:return super()._validate(message)
        if (set(message)!={'command','port','address','num','data','device'}
                or type(message['port']) is not int or message['port']!=1
                or type(message['device']) is not int or message['device']!=2
                or type(message['address']) is not int or type(message['num']) is not int
                or not isinstance(message['data'],list) or len(message['data'])!=2*message['num']
                or any(type(v) is not int or not 0<=v<=255 for v in message['data'])
                or not self.motion_enabled or self.terminal_latched or self._completed):
            raise ValueError('Invalid or unauthorized cycle target')
        values=tuple(message['data'][i]*256+message['data'][i+1] for i in range(0,len(message['data']),2))
        signature=(message['address'],values)
        plan=PLANS.get(getattr(self,'_cycle_phase',None),())
        if self._phase_index>=len(plan) or signature!=plan[self._phase_index]:raise ValueError('Target outside ordered position plan')
        # PREPARE at entry and deliberate supported release are separate actions;
        # each exact phase/index gets one attempt, including uncertain sends.
        key=(self._cycle_phase,self._phase_index,signature)
        if key in self._attempted_writes:raise RuntimeError('Cycle target already attempted; no retry')
        return key
