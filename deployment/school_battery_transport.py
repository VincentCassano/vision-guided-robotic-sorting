"""Isolated strict transport for supervised school battery hand poses."""
import json
import select
import socket
import time

READ_BLOCKS={(1000,8),(1085,6),(1095,6),(1105,5),(1115,5),
             (1125,6),(1135,6),(1145,6),(1165,6),(1175,5)}
MULTI='read_multiple_holding_registers'
SINGLE='read_holding_registers'
WRITE='write_registers'
ALLOWED_WRITES={(1125,(65535,)*6),(1135,(8000,8000,8000,8000,65535,65535)),
                (1135,(30000,)),(1135,(15000,15000,15000,15000,65535,65535)),
                (1135,(45000,30000,35000,40000,65535,65535)),
                (1135,(15000,30000,35000,40000,65535,65535)),
                (1136,(40000,)),(1135,(0,0,0,0,0,728)),
                (1135,(0,)),(1136,(0,0,0,0)),(1140,(728,))}
READ_BLOCKS |= {(1020,6),(1030,6),(1040,3),(1045,6),(1055,6),(1065,6),(1075,6),
                (1185,6),(1195,6),(1205,6),(1215,6)}

class TransportError(RuntimeError): pass

class Transport:
    def __init__(self,sock,log):
        self.sock,self.log=sock,log
        self.buffer=b'';self.sequence=0;self.broken=False;self.previous=None
        self.motion_enabled=False

    def receive(self):
        chunk=self.sock.recv(4096)
        self.log(dict(event='rx',sequence=self.sequence,monotonic=time.monotonic(),hex=chunk.hex()))
        if not chunk:raise TransportError('Connection closed')
        self.buffer+=chunk
        if len(self.buffer)>65536:raise TransportError('Oversized frame')

    def exchange(self,message):
        command=message.get('command')
        if (type(message.get('port')) is not int or type(message.get('device')) is not int
                or type(message.get('address')) is not int
                or message.get('port')!=1 or message.get('device')!=2):
            raise ValueError('Only hand port 1, node 2 allowed')
        if command==WRITE:
            data=message.get('data',[])
            if not self.motion_enabled or type(message.get('num')) is not int or len(data)!=2*message['num']:
                raise ValueError('Write not authorized')
            if any(type(v) is not int or not 0<=v<=255 for v in data):
                raise ValueError('Invalid write bytes')
            values=tuple(data[i]*256+data[i+1] for i in range(0,len(data),2))
            if (message.get('address'),values) not in ALLOWED_WRITES:
                raise ValueError('Write outside fixed school pose/speed allowlist')
        elif command==MULTI:
            if type(message.get('num')) is not int or (message.get('address'),message.get('num')) not in READ_BLOCKS:
                raise ValueError('Unapproved read')
        elif command==SINGLE:
            if message.get('address')!=1110:raise ValueError('Unapproved single read')
        else:raise ValueError('No robot motion/reset/configuration commands allowed')
        if self.broken:raise TransportError('Transport latched failed; no retries or writes')
        self.sequence+=1
        start=time.monotonic()
        try:
            if self.buffer:raise TransportError('Extra buffered response')
            if select.select([self.sock],[],[],.3)[0]:
                self.receive();raise TransportError('Unsolicited or late response before send')
            payload=(json.dumps(message,separators=(',',':'))+'\r\n').encode('ascii')
            self.log(dict(event='tx',sequence=self.sequence,monotonic=time.monotonic(),request=message,hex=payload.hex()))
            self.sock.sendall(payload)
            deadline=time.monotonic()+3
            while b'\n' not in self.buffer:
                left=deadline-time.monotonic()
                if left<=0:raise TransportError('Frame deadline exceeded')
                self.sock.settimeout(left);self.receive()
            line,self.buffer=self.buffer.split(b'\n',1)
            response=json.loads(line.decode('utf-8'))
            if self.buffer:raise TransportError('Duplicate/extra response in same receive')
            if not isinstance(response,dict) or response.get('command')!=message['command']:
                raise TransportError('Response command mismatch')
            if message['command']==WRITE:
                if response.get('write_state') is not True:raise TransportError('Write rejected or unacknowledged')
                value=None
            else:
                if response.get('read_state') is False:raise TransportError('Read rejected')
                data=response.get('data')
                if message['command']==SINGLE:
                    if type(data)!=int or not -32768<=data<=65535:raise TransportError('Invalid single register')
                    value=data & 65535
                else:
                    if not isinstance(data,list) or len(data)!=2*message['num'] or any(type(x)!=int or not 0<=x<=255 for x in data):
                        raise TransportError('Wrong response byte count/type')
                    value=[data[i]*256+data[i+1] for i in range(0,len(data),2)]
            self.log(dict(event='response',sequence=self.sequence,monotonic=time.monotonic(),
                          elapsed_s=time.monotonic()-start,response=response,value=value))
            self.previous=(message['command'],message.get('num'))
            return value
        except BaseException as exc:
            self.broken=True
            self.log(dict(event='transport_error',sequence=self.sequence,error=repr(exc),
                          pending_request=message,buffer_hex=self.buffer.hex()))
            raise

    def read(self,address,count):
        if (address,count) not in READ_BLOCKS:raise ValueError('Read not in allowlist')
        # Identity fence separates adjacent equal-length blocks, but cannot create
        # an address/transaction echo that the controller protocol does not have.
        if self.previous==(MULTI,count) and (address,count)!=(1000,8):
            identity=self.read(1000,8)
            if identity[0]!=512 or identity[5]!=2:raise TransportError('Identity fence mismatch')
        return self.exchange(dict(command=MULTI,port=1,address=address,num=count,device=2))

    def single(self,address):
        if address!=1110:raise ValueError('Single read not in allowlist')
        return self.exchange(dict(command=SINGLE,port=1,address=address,device=2))

    def enable_motion(self):
        if self.broken:raise TransportError('Failed transport cannot authorize writes')
        self.motion_enabled=True

    def disable_motion(self):self.motion_enabled=False

    def write_group(self,address,values):
        if type(address) is not int or (address,tuple(values)) not in ALLOWED_WRITES or any(type(v) is not int for v in values):
            raise ValueError('Not an approved school pose')
        return self.exchange(dict(command=WRITE,port=1,address=address,num=len(values),
            data=[b for v in values for b in (v>>8,v&255)],device=2))
