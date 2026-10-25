#!/usr/bin/env python3
"""One isolated, noninteractive native algorithm calculation. No robot socket."""
import argparse
import ctypes
import hashlib
import importlib
import json
import math
from pathlib import Path
import platform
import sys
import traceback
import types

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/battery_grasp/src'))
from battery_grasp.taught_path import compile_candidate,evaluate_candidates,NATIVE_SHA
from battery_grasp.taught_dh_model import reviewed_controller_model,derive_d1_conversion,coordinate_functions

ALLOWED_NATIVE=frozenset(('API_Version','Algo_Init_Sys_Data','Algo_Set_Angle','Algo_Get_Angle',
    'Algo_Set_WorkFrame','Algo_Get_Curr_WorkFrame','Algo_Set_ToolFrame','Algo_Get_Curr_ToolFrame',
    'Algo_Set_Joint_Min_Limit','Algo_Get_Joint_Min_Limit','Algo_Set_Joint_Max_Limit','Algo_Get_Joint_Max_Limit',
    'Algo_Forward_Kinematics','Algo_Inverse_Kinematics','libalgo_get_dh_value'))


class NumericalLibrary:
    """Expose only local algorithm symbols; Socket/Move/Set-controller rejected."""
    def __init__(self,native):self.__native=native
    def __getattr__(self,name):
        if name not in ALLOWED_NATIVE:raise RuntimeError('Forbidden native capability: '+name)
        return getattr(self.__native,name)


def encoded(value):
    if isinstance(value,ctypes.Structure):return {n:encoded(getattr(value,n)) for n,*_ in value._fields_}
    if isinstance(value,bytes):return value.decode('ascii')
    if isinstance(value,(tuple,list,ctypes.Array)):return [encoded(v) for v in value]
    return value


def frame(sdk,record):
    f=sdk.FRAME();f.frame_name.name=record['name'].encode('ascii')
    f.pose.position=sdk.Pos(*(record['position'][a] for a in ('x','y','z')))
    f.pose.euler=sdk.Euler(*(record['euler'][a] for a in ('rx','ry','rz')))
    rx,ry,rz=(record['euler'][a] for a in ('rx','ry','rz'))
    cr,sr=math.cos(rx/2),math.sin(rx/2);cp,sp=math.cos(ry/2),math.sin(ry/2);cy,sy=math.cos(rz/2),math.sin(rz/2)
    f.pose.quaternion=sdk.Quat(cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy)
    for a in ('payload','x','y','z'):setattr(f,a,record['payload'][a])
    return f


def same_frame(got,expected,kind):
    d=encoded(got)
    # Pinned 4.3.1 binary writes only position/euler for Work, plus payload for Tool.
    # It never writes the name or quaternion; those fields cannot certify the cache.
    for field,key in (('position','position'),('euler','euler')):
        if any(not math.isfinite(d['pose'][field][a]) or abs(d['pose'][field][a]-v)>1e-6 for a,v in expected[key].items()):
            raise ValueError('Local algorithm frame transform mismatch')
    if kind=='tool' and any(not math.isfinite(d[a]) or abs(d[a]-expected['payload'][a])>1e-6 for a in ('payload','x','y','z')):
        raise ValueError('Local algorithm frame payload metadata mismatch')


class NumericalAlgorithms:
    """Explicit pointer/return ABI for the pinned library, independent of legacy wrappers."""
    def __init__(self,sdk):
        self.sdk=sdk;self.native=NumericalLibrary(sdk.Arm.pDll)
        if (ctypes.sizeof(sdk.Pose)!=40 or ctypes.sizeof(sdk.FRAME)!=68
                or ctypes.alignment(sdk.Pose)!=4 or ctypes.alignment(sdk.FRAME)!=4
                or [getattr(sdk.Pose,n).offset for n in ('position','quaternion','euler')]!=[0,12,28]
                or [getattr(sdk.FRAME,n).offset for n in ('frame_name','pose','payload','x','y','z')]!=[0,12,52,56,60,64]):
            raise ValueError('Pinned native Pose/FRAME layout mismatch')
        pf=ctypes.POINTER(ctypes.c_float);pframe=ctypes.POINTER(sdk.FRAME)
        signatures={'API_Version':([],ctypes.c_char_p),'Algo_Init_Sys_Data':([ctypes.c_int,ctypes.c_int],None),
            'Algo_Set_Angle':([ctypes.c_float]*3,None),'Algo_Get_Angle':([pf]*3,None),
            'Algo_Forward_Kinematics':([pf],sdk.Pose),
            'Algo_Inverse_Kinematics':([pf,ctypes.POINTER(sdk.Pose),pf,ctypes.c_uint8],ctypes.c_int)}
        for kind in ('Work','Tool'):
            signatures['Algo_Set_'+kind+'Frame']=([pframe],None)
            signatures['Algo_Get_Curr_'+kind+'Frame']=([pframe],None)
        for lowhigh in ('Min','Max'):
            for action in ('Set','Get'):signatures['Algo_'+action+'_Joint_'+lowhigh+'_Limit']=([pf],None)
        for name,(args,result) in signatures.items():
            function=getattr(self.native,name);function.argtypes=args;function.restype=result

    def __getattr__(self,name):
        fn=getattr(self.native,name)
        def invoke(*args):
            if name=='Algo_Get_Angle':
                values=[ctypes.c_float() for _ in range(3)];fn(*(ctypes.byref(v) for v in values))
                return [v.value for v in values]
            if name in ('Algo_Set_WorkFrame','Algo_Set_ToolFrame'):return fn(ctypes.byref(args[0]))
            if name in ('Algo_Get_Curr_WorkFrame','Algo_Get_Curr_ToolFrame'):
                value=self.sdk.FRAME();fn(ctypes.byref(value));return value
            if name.startswith('Algo_Set_Joint_'):return fn((ctypes.c_float*7)(*args[0]))
            if name.startswith('Algo_Get_Joint_'):
                values=(ctypes.c_float*7)();fn(values);return list(values)
            if name=='Algo_Forward_Kinematics':
                value=fn((ctypes.c_float*7)(*args[0]))
                return [value.position.x,value.position.y,value.position.z,value.euler.rx,value.euler.ry,value.euler.rz]
            if name=='Algo_Inverse_Kinematics':
                seed,pose,flag=args
                if flag!=1:raise ValueError('Only audited Euler IK input supported')
                value=self.sdk.Pose();value.position=self.sdk.Pos(*pose[:3]);value.euler=self.sdk.Euler(*pose[3:])
                output=(ctypes.c_float*7)();status=fn((ctypes.c_float*7)(*seed),ctypes.byref(value),output,flag)
                return status,list(output)
            return fn(*args)
        return invoke


def configure(sdk,candidate,emit,controller_model=None):
    numerical=NumericalAlgorithms(sdk)
    def call(method,*args):
        emit(dict(event='algorithm_call_before',method=method,args=encoded(args)))
        answer=getattr(numerical,method)(*args)
        emit(dict(event='algorithm_call_result',method=method,result=encoded(answer)))
        return answer
    # RobotType.RM65=0 is the algorithm enum. The socket constructor constant RM65=65 is NOT used.
    call('Algo_Init_Sys_Data',sdk.RobotType.RM65,sdk.SensorType.B)
    version=numerical.native.API_Version().decode('ascii');emit(dict(event='native_version',version=version))
    if version!='4.3.1':raise ValueError('Expected the installed native API4.3.1')
    call('Algo_Set_Angle',*candidate['algorithm_installation_degrees'])
    actual=call('Algo_Get_Angle')
    if len(actual)!=3 or any(abs(a-b)>1e-5 for a,b in zip(actual,candidate['algorithm_installation_degrees'])):
        raise ValueError('Local algorithm installation angles do not match measured settings')
    for kind in ('work','tool'):
        supplied=candidate['frame_signatures'][kind]
        suffix='WorkFrame' if kind=='work' else 'ToolFrame'
        probe=dict(name='LocalProbe',position=dict(x=.011,y=-.013,z=.017),euler=dict(rx=.07,ry=-.05,rz=.09),
            payload=dict(payload=.125,x=.001,y=-.002,z=.003))
        emit(dict(event='local_frame_probe',kind=kind,scope='offline algorithm cache only; no device connection',
            returned_fields=['position','euler']+(['payload','x','y','z'] if kind=='tool' else []),
            unavailable_fields=['frame_name','quaternion']+(['payload','x','y','z'] if kind=='work' else [])))
        try:
            call('Algo_Set_'+suffix,frame(sdk,probe))
            same_frame(call('Algo_Get_Curr_'+suffix),probe,kind)
        finally:
            call('Algo_Set_'+suffix,frame(sdk,supplied))
            same_frame(call('Algo_Get_Curr_'+suffix),supplied,kind)
    for lowhigh,key in (('Min','minimum'),('Max','maximum')):
        expected=candidate['ordinary_limits'][key]
        call('Algo_Set_Joint_'+lowhigh+'_Limit',expected)
        actual=call('Algo_Get_Joint_'+lowhigh+'_Limit')
        if len(actual)!=7 or any(abs(a-b)>1e-5 for a,b in zip(actual[:6],expected)):
            raise ValueError('Local algorithm joint limits mismatch')
    fk,ik=numerical.Algo_Forward_Kinematics,numerical.Algo_Inverse_Kinematics
    if controller_model is not None:
        # Pinned binary symbol only copies its 256-byte local DH block. No setter.
        get=numerical.native.libalgo_get_dh_value
        get.argtypes=[ctypes.POINTER(ctypes.c_float)];get.restype=None
        values=(ctypes.c_float*64)()
        emit(dict(event='local_nominal_DH_read_before',symbol='libalgo_get_dh_value',buffer_bytes=256))
        get(values)
        emit(dict(event='local_nominal_DH_read_result',values=list(values)))
        conversion=derive_d1_conversion(controller_model,list(values),candidate)
        emit(dict(event='measured_DH_model_conversion',**conversion))
        fk,ik=coordinate_functions(fk,ik,conversion,emit)
    emit(dict(event='local_algorithm_configuration_verified',nonzero_frame_probes_restored=True,
        measured_controller_DH_used=controller_model is not None,
        frame_names_not_returned_by_native=True,network_connection_requested=False))
    return fk,ik


def load_reviewed_sdk(directory,emit):
    if platform.machine()!='aarch64' or platform.system()!='Linux':
        raise ValueError('Numerical native stage requires the original onsite Linux aarch64 host')
    expected=json.loads((ROOT/'deployment/arm_sdk_reviewed_hashes.json').read_text(encoding='utf-8'))
    for name,digest in expected.items():
        if hashlib.sha256((directory/name).read_bytes()).hexdigest()!=digest:
            raise ValueError('Reviewed SDK Python source changed: '+name)
    binary=directory/'libRM_Base.so';data=binary.read_bytes()
    if hashlib.sha256(data).hexdigest()!=NATIVE_SHA:raise ValueError('Onsite native library differs from captured SHA')
    emit(dict(event='native_identity',directory=str(directory),bytes=len(data),sha256=NATIVE_SHA,
        robot_type='RM65 nominal model; checked against all recorded actual joint/TCP witnesses',
        sensor_type='B nominal algorithm setting; not a fresh physical sensor identity read',
        network_connection_requested=False,robot_instance_constructed=False))
    package=types.ModuleType('battery_taught_path_numerical_sdk');package.__path__=[str(directory)]
    sys.modules[package.__name__]=package
    return importlib.import_module(package.__name__+'.robotic_arm')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path);parser.add_argument('--sdk-dir',required=True,type=Path)
    args=parser.parse_args();folder=args.output.resolve()
    report=dict(schema_version=1,worker_completed=False,execution_ready=False,hardware_writes=0,
        robot_instance_constructed=False,network_connection_requested=False,exceptions=[])
    def save():
        (folder/'native_result.json').write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')
    with (folder/'algorithm_calls.jsonl').open('w',encoding='utf-8') as log:
        def emit(row):log.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n');log.flush()
        try:
            candidate=compile_candidate((ROOT/'deployment/battery_teaching_212012.json').read_bytes())
            model_raw=(ROOT/'deployment/taught_model_parameters_192049.json').read_bytes()
            controller_model=reviewed_controller_model(model_raw)
            report['algorithm_model_source']=controller_model
            (folder/'model_parameters_source.json').write_bytes(model_raw)
            sdk=load_reviewed_sdk(args.sdk_dir.resolve(),emit)
            fk,raw_ik=configure(sdk,candidate,emit,controller_model)
            assessment=evaluate_candidates(candidate,fk,lambda q,p:raw_ik(q,p,1),emit)
            report.update(assessment,worker_completed=True)
        except Exception as exc:
            report['exceptions'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
            print(traceback.format_exc(),file=sys.stderr)
        finally:save()
    return 0 if report['worker_completed'] else 1


if __name__=='__main__':raise SystemExit(main())
