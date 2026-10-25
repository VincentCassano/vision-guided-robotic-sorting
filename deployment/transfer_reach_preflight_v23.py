"""Fresh-device geometric reach screening for the actual transfer targets.

No native algorithm initialization, controller setter, FK/IK solver or test
entry. This necessary shoulder-to-wrist bound does not certify joint limits
or collision-free trajectories; the existing controller and monitors remain.
"""
import json
import math
import socket
import time
from pathlib import Path
from arm_protection_transport import unique_object
from battery_grasp.taught_dh_model import reviewed_controller_model

ROOT=Path(__file__).resolve().parents[1]


def collect_controller_model(emit,check_abort):
    replies={}
    for command in ('get_arm_software_info','get_DH_data'):
        check_abort();sock=None;buffer=b'';deadline=time.monotonic()+3.
        def remaining():
            value=deadline-time.monotonic()
            if value<=0:raise TimeoutError('Controller model GET deadline: '+command)
            return value
        try:
            sock=socket.create_connection(('169.254.128.18',8080),timeout=remaining())
            request={'command':command};payload=(json.dumps(request,separators=(',',':'))+'\r\n').encode('ascii')
            emit(dict(event='model_GET_tx',request=request,hex=payload.hex(),monotonic=time.monotonic()))
            sock.settimeout(remaining());sock.sendall(payload)
            while b'\r\n' not in buffer:
                sock.settimeout(remaining());part=sock.recv(4096)
                emit(dict(event='model_GET_rx',command=command,hex=part.hex(),monotonic=time.monotonic()))
                if not part:raise OSError('Controller model GET EOF')
                buffer+=part
                if len(buffer)>65536:raise ValueError('Controller model reply oversized')
            line,extra=buffer.split(b'\r\n',1)
            if extra:raise ValueError('Trailing controller model reply')
            response=json.loads(line.decode('utf8'),object_pairs_hook=unique_object,
                parse_constant=lambda x:(_ for _ in ()).throw(ValueError('Nonfinite controller model')))
            replies[command]=response;emit(dict(event='model_GET_response',command=command,response=response))
        finally:
            if sock is not None:sock.close()
    model=reviewed_controller_model((ROOT/'deployment/taught_model_parameters_192049.json').read_bytes())
    identity=replies['get_arm_software_info'];dh=replies['get_DH_data']
    if (identity.get('command')!='arm_software_info' or identity.get('Product_version')!='RM65-BI'
            or dh.get('command')!='get_DH_data'
            or [dh.get('joint_'+str(i)) for i in range(1,7)]!=model['rows']):
        raise ValueError('Fresh controller identity/DH differs from reviewed model')
    return dict(fresh_GETs=replies,reviewed_controller_model=model,hardware_configuration_writes=0)


class TransferReachPreflight:
    def __init__(self,live,installation,model_read,emit,check_abort):
        self.emit=emit;self.check_abort=check_abort
        if len(installation)!=3 or any(abs(a-b)>.001 for a,b in zip(installation,[0.,45.,0.])):
            raise ValueError('Reach bound requires the actual0/45/0 installation')
        for kind in ('work','tool'):
            value=live['frame_signatures'][kind]
            if any(abs(v)>1e-9 for field in ('position','euler') for v in value[field].values()):
                raise ValueError('Reach bound requires actual zero World/Arm_Tip transforms')
        rows=model_read['reviewed_controller_model']['rows']
        d1=rows[0][2]/1e6;self.radius=(rows[2][1]+rows[3][2])/1e6;self.d6=rows[5][2]/1e6
        self.shoulder=[math.sin(math.pi/4)*d1,0.,math.cos(math.pi/4)*d1]
        self.check(live['pose'],'fresh_actual_pose')

    def check(self,pose,label):
        self.check_abort()
        if len(pose)!=6 or any(type(v) not in (int,float) or not math.isfinite(v) for v in pose):
            raise ValueError('Finite transfer pose required')
        rx,ry,rz=pose[3:]
        z=[math.cos(rz)*math.sin(ry)*math.cos(rx)+math.sin(rz)*math.sin(rx),
           math.sin(rz)*math.sin(ry)*math.cos(rx)-math.cos(rz)*math.sin(rx),math.cos(ry)*math.cos(rx)]
        wrist=[pose[i]-self.d6*z[i] for i in range(3)]
        distance=math.dist(wrist,self.shoulder)
        row=dict(phase=label,pose_m_rad=pose[:],wrist_center_world_m=wrist,
            shoulder_wrist_mm=distance*1000,maximum_link_sum_mm=self.radius*1000,
            radial_margin_mm=(self.radius-distance)*1000,
            necessary_geometric_bound_only=True,IK_or_collision_certificate=False)
        self.emit(dict(event='transfer_reach_bound',**row))
        if distance>self.radius+1e-9:
            raise ValueError('Transfer target outside geometric reach: '+label+' excess_mm='+str((distance-self.radius)*1000))
        return row

    def check_rows(self,rows,post_drop_raise_mm):
        values=[self.check(row['pose'],row['phase']) for row in rows]
        exit_pose=rows[-1]['pose'][:];exit_pose[2]+=post_drop_raise_mm/1000.
        values.append(self.check(exit_pose,'post_drop_clearance_endpoint'))
        return dict(points=values,all_transfer_and_exit_targets_checked_before_motion=True,
            necessary_geometric_bound_only=True,controller_solver_unchanged=True)
