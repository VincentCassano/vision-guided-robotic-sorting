"""Read SDK provenance/GET metadata only, without extra loading or connections."""
import ctypes
import hashlib
from pathlib import Path
import platform
import sys

from inspect_left_arm import encode


def binary_identity(data):
    if data[:4] == b'\x7fELF' and len(data)>=20 and data[4] in (1,2) and data[5] in (1,2):
        byteorder='little' if data[5]==1 else 'big'
        machine=int.from_bytes(data[18:20],byteorder)
        return dict(format='ELF',bits=32 if data[4]==1 else 64,byteorder=byteorder,
                    machine=machine,architecture={62:'x86_64',183:'aarch64',3:'x86',40:'arm'}.get(machine,'unknown'))
    if data[:2]==b'MZ' and len(data)>=64:
        offset=int.from_bytes(data[60:64],'little')
        if offset+6<=len(data) and data[offset:offset+4]==b'PE\x00\x00':
            machine=int.from_bytes(data[offset+4:offset+6],'little')
            return dict(format='PE',machine=machine,architecture={0x8664:'x86_64',0x14c:'x86',0xaa64:'aarch64'}.get(machine,'unknown'))
    raise ValueError('Unrecognized/truncated ELF or PE header')


def collect_sdk_environment(robot):
    result=dict(schema_version=1,read_only=True,execution_ready=False,
        python_version=sys.version,host_system=platform.system(),host_machine=platform.machine(),
        python_pointer_bits=ctypes.sizeof(ctypes.c_void_p)*8,errors=[],
        note='Provenance only. Does not certify the native algorithm or approve IK/trajectory execution.')
    try:
        name=robot.pDll._name
        if not isinstance(name,str) or not Path(name).is_absolute():
            raise ValueError('Loaded native path is not an absolute file path')
        path=Path(name)
        data=path.read_bytes()
        result['loaded_native']=dict(path=str(path),bytes=len(data),sha256=hashlib.sha256(data).hexdigest(),
                                     binary=binary_identity(data))
    except Exception as exc:
        result['errors'].append(dict(field='loaded_native',error=repr(exc)))
    try:
        version=robot.API_Version()
        if isinstance(version,bytes):version=version.decode('ascii')
        if not isinstance(version,str) or not version.strip():
            raise ValueError('Invalid API version return')
        result['api_version']=version
    except Exception as exc:
        result['errors'].append(dict(field='api_version',error=repr(exc)))
    for name,field in (('Get_Arm_Software_Info','controller_software_info'),('Get_Arm_Run_Mode','controller_run_mode_raw')):
        try:
            answer=getattr(robot,name)()
            if not isinstance(answer,(tuple,list)) or len(answer)<2 or type(answer[0]) is not int:
                raise ValueError('Malformed GET response')
            result[field]=dict(status=answer[0],ok=answer[0]==0,data=encode(answer[1:]) if answer[0]==0 else None)
            if answer[0]!=0:raise ValueError('GET unsuccessful: '+str(answer[0]))
        except Exception as exc:
            result['errors'].append(dict(field=field,error=repr(exc)))
    result['metadata_complete']=not result['errors']
    return result
