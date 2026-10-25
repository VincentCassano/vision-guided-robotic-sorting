"""Measured DH to pinned nominal RM65 numerical coordinates; never device writes.

Only D1 differs. In Craig modified DH, alpha1=a1=offset1=0,
so D1 is a fixed translation along installed base Z, before joint rotation.
FK adds R_install*[0,0,(D1_controller-D1_nominal)]. IK subtracts it.
All other DH values, zero frames and the exact installation must agree first.
The transform is computed solely from read-back DH, never fitted to TCP points.
"""
import hashlib
import json
import math

MODEL_SHA='5067ef8bbc7fa921b684ad9c35ecf47641a044b6d4281c6f18574ba32b64fe43'
EXPECTED_DH=[[0,0,181060,0],[90000,0,0,90000],[0,256000,0,90000],
             [90000,0,210000,0],[-90000,0,0,0],[90000,0,144000,0]]


def reviewed_controller_model(raw):
    if hashlib.sha256(raw).hexdigest()!=MODEL_SHA:raise ValueError('Unreviewed controller model source')
    value=json.loads(raw)
    if (value.get('dh_valid') is not True or value.get('identity_valid') is not True
            or value.get('errors')!=[] or value.get('hardware_writes')!=0
            or value.get('controller_configuration_writes')!=0 or value.get('actuator_writes')!=0
            or value.get('model_data_applied') is not False or value.get('fitted_correction_applied') is not False
            or value.get('parameters_origin')!='controller_readback_not_manufacturer_default'
            or value.get('execution_ready') is not False or value.get('endpoint')!=['169.254.128.18',8080]
            or value.get('identity_response',{}).get('Product_version')!='RM65-BI'):
        raise ValueError('Controller model identity/provenance differs')
    transactions=value['transactions']
    if len(transactions)!=2:raise ValueError('Model transaction count differs')
    for transaction,command,response in zip(transactions,('get_arm_software_info','get_DH_data'),
            (value['identity_response'],value['dh_response'])):
        expected=(json.dumps({'command':command},separators=(',',':'))+'\r\n').encode('ascii')
        if (bytes.fromhex(transaction['request_hex'])!=expected or transaction['command']!=command
                or transaction['request_send_completed'] is not True or transaction['socket_closed'] is not True
                or transaction['response']!=response
                or json.loads(bytes.fromhex(transaction['response_hex']))!=response):
            raise ValueError('Controller model raw request/response differs')
    rows=[value['dh_response']['joint_'+str(i)] for i in range(1,7)]
    if rows!=EXPECTED_DH:raise ValueError('Reviewed six-joint DH data differs')
    return dict(source_sha256=MODEL_SHA,rows=rows,identity=value['identity_response'],
        source='controller readback 2026-10-12 192049; not manufacturer defaults or fitted TCP offsets')


def finite_vector(values,count,name):
    if not isinstance(values,(list,tuple)) or len(values)!=count:
        raise ValueError(name+' vector length differs')
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in values):
        raise ValueError(name+' requires finite numeric values')
    return list(values)


def derive_d1_conversion(controller,nominal_block,candidate):
    nominal=finite_vector(nominal_block,64,'Pinned local DH block')
    if controller.get('source_sha256')!=MODEL_SHA or controller.get('rows')!=EXPECTED_DH:
        raise ValueError('Reviewed controller DH required')
    if candidate['algorithm_installation_degrees']!=[0.0,45.0,0.0]:
        raise ValueError('D1 mapping requires audited installation [0,45,0]')
    for kind in ('work','tool'):
        frame=candidate['frame_signatures'][kind]
        if any(v!=0 for field in ('position','euler') for v in frame[field].values()):
            raise ValueError('D1 mapping requires audited zero work/tool transforms')
    if any(v!=0 for v in nominal[32:]):
        raise ValueError('Unexpected native DH compensation already present')
    if any(nominal[offset+i]!=0 for offset in (0,8,16,24) for i in (6,7)):
        raise ValueError('Unexpected inactive DH entries')
    measured=[];nominal_rows=[]
    for i,row in enumerate(controller['rows']):
        actual=[math.radians(row[0]/1000),row[1]/1e6,row[2]/1e6,math.radians(row[3]/1000)]
        default=[nominal[16+i],nominal[8+i],nominal[i],nominal[24+i]]
        for k,(a,b) in enumerate(zip(actual,default)):
            if not (i==0 and k==2) and abs(a-b)>1e-7:
                raise ValueError('Other DH parameter differs; D1-only mapping forbidden')
        measured.append(actual);nominal_rows.append(default)
    if measured[0][0]!=0 or measured[0][1]!=0 or measured[0][3]!=0:
        raise ValueError('First modified-DH joint is not pure base-Z displacement')
    if abs(nominal[0]-.2405)>1e-7:raise ValueError('Pinned RM65 nominal D1 differs')
    delta=measured[0][2]-nominal[0]
    shift=[math.sin(math.pi/4)*delta,0.0,math.cos(math.pi/4)*delta]
    return dict(source_sha256=MODEL_SHA,mode='measured_D1_parameter_coordinate_conversion',
        nominal_rows_alpha_a_d_offset_rad_m=nominal_rows,controller_rows_alpha_a_d_offset_rad_m=measured,
        nominal_D1_mm=nominal[0]*1000,controller_D1_mm=measured[0][2]*1000,D1_delta_mm=delta*1000,
        nominal_to_controller_world_shift_m=shift,
        derivation='modified DH alpha1=a1=offset1=0; R_install*[0,0,D1_controller-D1_nominal]',
        fitted_from_tcp=False,native_DH_mutated=False,controller_parameters_mutated=False,
        whole_robot_model_certified=False,execution_ready=False)


def coordinate_functions(fk,ik,conversion,emit):
    shift=conversion['nominal_to_controller_world_shift_m']
    def forward(q):
        nominal=finite_vector(fk(q),6,'Nominal FK pose')
        measured=[a+b for a,b in zip(nominal[:3],shift)]+nominal[3:]
        emit(dict(event='measured_DH_forward_conversion',joint_degrees=list(q),
            nominal_pose_m_rad=nominal,controller_DH_pose_m_rad=measured))
        return measured
    def inverse(q,pose,flag=1):
        target=finite_vector(pose,6,'Measured DH IK target')
        nominal=[a-b for a,b in zip(target[:3],shift)]+target[3:]
        emit(dict(event='measured_DH_inverse_conversion',seed_degrees=list(q),
            controller_DH_target_m_rad=target,nominal_target_m_rad=nominal,flag=flag))
        return ik(q,nominal,flag)
    return forward,inverse


def modified_dh_matrix(q_degrees,rows,install_y_degrees=45.0):
    """Independent FK oracle for offline validation; no fitting or inverse solver."""
    q=finite_vector(q_degrees,6,'MDH joints');b=math.radians(install_y_degrees)
    transform=[[math.cos(b),0,math.sin(b),0],[0,1,0,0],[-math.sin(b),0,math.cos(b),0],[0,0,0,1]]
    for joint,row in zip(q,rows):
        alpha,a,d,offset=row;theta=math.radians(joint)+offset
        c,s,ca,sa=math.cos(theta),math.sin(theta),math.cos(alpha),math.sin(alpha)
        link=[[c,-s,0,a],[s*ca,c*ca,-sa,-sa*d],[s*sa,c*sa,ca,ca*d],[0,0,0,1]]
        transform=[[sum(transform[i][k]*link[k][j] for k in range(4)) for j in range(4)] for i in range(4)]
    return transform
