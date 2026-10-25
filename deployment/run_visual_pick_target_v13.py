"""Capture a visual point for a new wrist reference; no old TCP mapping or robot access."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys
import time
import traceback
from known_cylinder_records import archive_folder, dump

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/battery_grasp/src'))


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def bbox_iou(a,b):
    ax,ay,aw,ah=a;bx,by,bw,bh=b
    intersection=max(0,min(ax+aw,bx+bw)-max(ax,bx))*max(0,min(ay+ah,by+bh)-max(ay,by))
    return intersection/max(aw*ah+bw*bh-intersection,1e-9)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--position-session',type=Path,required=True)
    parser.add_argument('--confirm-fixed-camera',action='store_true',required=True)
    parser.add_argument('--confirm-measured-shell-only',action='store_true',required=True)
    parser.add_argument('--inference-python',default='/usr/bin/python3')
    parser.add_argument('--device',default='auto')
    parser.add_argument('--confirm-unchanged-setup',action='store_true',required=True)
    parser.add_argument('--anchor',type=Path,required=True,help='Reviewed historical visual evidence for equal-height plane geometry only')
    parser.add_argument('--grasp-source',type=Path,default=ROOT/'deployment/visual_binding_grasp_source_123051.json')
    parser.add_argument('--pregrasp-source',type=Path,default=ROOT/'deployment/visual_binding_pregrasp_source_024412.json')
    parser.add_argument('--pregrasp-crosscheck',type=Path,default=ROOT/'deployment/visual_binding_pregrasp_crosscheck_081023.json')
    parser.add_argument('--confirm-upright-equal-height-recesses',action='store_true',required=True)
    parser.add_argument('--finish-after-first-reference',action='store_true')
    args=parser.parse_args()
    folder=ROOT/'captures'/('visual_pick_target_v13_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True,exist_ok=False)
    pipeline=None;client=None;started=False;saved=0;capture=None;points=[];binding=None;target_count=0
    last_capture_message=None;last_capture_pass=None;plane=None
    report=dict(scope='live_visual_pick_target_point_only',
        delivery_version='visual-pick-target-v13-20261023',old_grasp_tcp_mapping_used=False,
        old_2p8_plus_4p5_offsets_used=False,new_wrist_pose_read_by_this_process=False,
        robot_SDK_loaded=False,arm_connections=0,arm_writes=0,hand_writes=0,
        controller_configuration_writes=0,camera_option_writes=0,execution_ready=False,
        full6D_handeye_pass=False,RGBD_position_validation_pass=False,
        object_geometry_independently_accepted=False,source_session=str(args.position_session))
    try:
        from visual_task_anchor import accepted_grasp,validate_saved_anchor
        from visual_equal_height_plane import make_plane,project_observation,plane_summary
        grasp=accepted_grasp(args.grasp_source.read_bytes(),args.pregrasp_source.read_bytes(),
            args.pregrasp_crosscheck.read_bytes())
        report['historical_grasp_metadata_role']='only_validate_original_visual_plane_provenance_not_new_wrist_or_TCP'
        report['historical_grasp_metadata']=grasp
        candidate_path=ROOT/'deployment/camera_position_candidate_225543.json'
        candidate=json.loads(candidate_path.read_text(encoding='utf8'));candidate_sha=sha(candidate_path)
        qualification=json.loads((ROOT/'deployment/marker_pnp_position_005012_evidence.json').read_text(encoding='utf8'))
        session=args.position_session.resolve()
        validation=json.loads((session/'position_validation_result.json').read_text(encoding='utf8'))
        if (candidate_sha!=qualification['candidate_sha256'] or validation['candidate_sha256']!=candidate_sha
                or qualification['marker_pnp_position_consistency_pass'] is not True
                or candidate['fit_count']!=16 or candidate['fit_position_pass'] is not True
                or candidate['excluded_samples'] or candidate['validation_used_for_fit']):
            raise ValueError('Frozen position candidate and reviewed marker evidence required')
        rows=validation['validation']
        if (len(rows)!=6 or {r['id'] for r in rows}!={'001','002','003','004','005','006'}
                or any(not 0<=r['position_error_pnp_mm']<=5 for r in rows)
                or validation.get('source_candidate_changed') or validation.get('validation_used_for_fit')
                or validation.get('error')):
            raise ValueError('All six original marker-PnP positions required; no promotion of RGBD gate')
        for name,digest in qualification['validation_source_hashes'].items():
            target=(session/name).resolve()
            if target.parent!=session or sha(target)!=digest:raise ValueError('Reviewed evidence changed: '+name)
        model=json.loads((ROOT/'deployment/black_cylinder_measured_20261020.json').read_text(encoding='utf8'))
        weights=ROOT/'src/battery_grasp/models/battery_v3_camera.torchscript'
        if not weights.is_file():raise FileNotFoundError('Keep the existing accepted TorchScript model: '+str(weights))
        import cv2
        import numpy as np
        import pyrealsense2 as rs
        import yaml
        from battery_grasp.inference_client import InferenceClient
        from known_cylinder_geometry_axis import locate
        import known_cylinder_capture as burst
        thresholds=yaml.safe_load((ROOT/'src/battery_grasp/config/yolo_v3_calibrated.yaml').read_text())['battery_detector_yolo']['class_confidences']
        report.update(candidate_sha256=candidate_sha,marker_position_qualification=qualification,
            measured_shell=model,weights_sha256=sha(weights),class_confidences=thresholds,
            new_reference_place_confirmation_required_in_parent=True,historical_plane_geometry_reused=True,
            new_wrist_reference_point_must_be_captured_this_run=True,
            runtime_source_sha256={name:sha(ROOT/'deployment'/name) for name in
                ('run_visual_pick_target_v13.py','visual_task_anchor.py','known_cylinder_geometry_axis.py',
                 'known_cylinder_capture.py','known_cylinder_records.py','visual_equal_height_plane.py')})
        client=InferenceClient(args.inference_python,str(ROOT/'src/battery_grasp/scripts/yolo_worker.py'),
            str(ROOT/'deployment/yolov5-runtime'),str(weights),device=args.device,class_confidences=thresholds)
        pipeline=rs.pipeline();config=rs.config();config.enable_device('344322075008')
        config.enable_stream(rs.stream.color,1280,720,rs.format.bgr8,30)
        profile=pipeline.start(config);started=True
        intr=profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        k=np.array([[intr.fx,0,intr.ppx],[0,intr.fy,intr.ppy],[0,0,1.]],float)
        if (k.tolist()!=candidate['source_context']['K'] or list(intr.coeffs)!=candidate['source_context']['distortion']
                or str(intr.model)!=candidate['source_context']['distortion_model'] or any(intr.coeffs)):
            raise ValueError('Camera intrinsics changed; no assumed compensation')
        transform=np.asarray(candidate['world_from_camera'],float)
        context=dict(camera_serial='344322075008',image_size=[1280,720],K=k.tolist(),
            distortion=list(intr.coeffs),distortion_model=str(intr.model),candidate_sha256=candidate_sha,
            weights_sha256=sha(weights),shell_model_sha256=sha(ROOT/'deployment/black_cylinder_measured_20261020.json'),
            source_grasp_sha256=grasp['source_worker_sha256'],camera_fixed_confirmed=args.confirm_fixed_camera,
            source_pregrasp_sha256=grasp['pregrasp_source_sha256'],
            source_pregrasp_crosscheck_sha256=grasp['pregrasp_crosscheck_sha256'],
            measured_shell=dict(diameter_mm=model['diameter_mm'],total_height_mm=model['total_height_mm']))
        context['geometry_module_sha256']=sha(ROOT/'deployment/known_cylinder_geometry_axis.py')
        report['binding_camera_context']=context
        report['cap_measurement_method']='body_axis_aligned_upper_silhouette_fallback; original_closed_rim_first'
        report['capture_input_policy']='freeze_first_selected_detector_bbox_for_entire_burst; preserve_raw_detector_bbox_and_every_RGB_frame'
        report['per_frame_coordinates_smoothed']=False
        report['original_per_frame_image_geometry_gates_retained']=True
        report['original_free3D_burst_verdict_separately_recorded']=True
        report['new_equal_height_plane_method_not_free3D_acceptance']=True
        report['plane_burst_repeatability_mm']=3.0
        if args.anchor is not None:
            anchor_path=args.anchor.resolve()
            binding=validate_saved_anchor(json.loads(anchor_path.read_bytes()),context,grasp)
            for stem in ('reference_observation','reference_image'):
                name=binding[stem+'_file'];source_path=(anchor_path.parent/name).resolve()
                if (source_path.parent!=anchor_path.parent or source_path.name!=name
                        or sha(source_path)!=binding[stem+'_sha256']):
                    raise ValueError('Saved reference evidence differs: '+name)
                copied_name='geometry_source_'+stem+source_path.suffix
                (folder/copied_name).write_bytes(source_path.read_bytes())
                report.setdefault('historical_plane_geometry_evidence',{})[stem]=dict(
                    file=copied_name,sha256=sha(source_path),original_name=name)
                if stem=='reference_observation':geometry_reference_observation=json.loads(source_path.read_bytes())
            dump(folder/'geometry_source_anchor.json',binding)
            report.update(historical_geometry_source_validated=True,geometry_source_anchor=str(anchor_path),
                geometry_source_anchor_sha256=sha(anchor_path),old_grasp_tcp_mapping_used=False)
        if binding is None:raise ValueError('Reviewed visual plane geometry source required')
        plane=make_plane(binding,geometry_reference_observation,
            transform,args.confirm_upright_equal_height_recesses)
        plane['method_source_sha256']=sha(ROOT/'deployment/visual_equal_height_plane.py')
        plane['original_task_anchor_sha256']=sha(folder/'geometry_source_anchor.json')
        plane['old_grasp_tcp_mapping_used']=False
        dump(folder/'equal_height_plane.json',plane)
        report['equal_height_plane']=plane
        report['equal_height_plane_sha256']=sha(folder/'equal_height_plane.json')
        title='VISUAL PICK TARGET | S save same seated shell | Q finish | NO MOTION'
        cv2.namedWindow(title,cv2.WINDOW_NORMAL)
        selection=dict(bbox=None,click=None)
        def mouse(event,x,y,flags,param):
            if event==cv2.EVENT_LBUTTONDOWN and y>=100:selection['click']=(x,y-100)
        cv2.setMouseCallback(title,mouse)
        print('VISUAL PICK TARGET：点选现在要抓的落槽黑色空壳，按S保存本轮新目标；不计算旧腕姿TCP或旧修正量。',flush=True)
        print('旧3D结果和旧3mm判定单独保留；新平面方法仍需8帧/2秒/3mm。不查槽号、不平滑坐标或删除波动帧。',flush=True)
        print('使用已经保存的新腕姿绑定；在这里点选当前要抓的落槽空壳，按S。采图后空壳/头部/工作板不能移动。',flush=True)
        print('只复用已验收同高平面的几何依据；不复用旧抓取位、腕姿或2.8+4.5mm偏移。仅同一空壳直立落底、同高工作面。',flush=True)
        print('原3D判定另存，平面法通过也不改写3D失败。每帧原图统一保留，Q一个ZIP；不连接SDK或移动臂手。',flush=True)
        while True:
            frames=pipeline.wait_for_frames(5000);color=frames.get_color_frame()
            if not color:continue
            image=np.asanyarray(color.get_data()).copy()
            detections=client.detect(image)
            black=[d for d in detections if d['class_id']==0 and d['confidence']>=thresholds[0]]
            result=dict(delivery_version=report['delivery_version'],
                frame_number=int(color.get_frame_number()),timestamp_ms=float(color.get_timestamp()),
                detections=detections,geometry_candidate_available=False,execution_ready=False,
                selection_scope='same_seated_shell_visual_point_for_new_wrist_reference',
                old_grasp_tcp_mapping_used=False,old_2p8_plus_4p5_offsets_used=False,motion_authorized=False)
            view=image.copy()
            for d in detections:
                x,y,w,h=[int(v) for v in d['bbox']]
                cv2.rectangle(view,(x,y),(x+w,y+h),(140,140,140),1)
                cv2.putText(view,'%s %.2f'%(d['class_name'],d['confidence']),(max(0,x),max(15,y-5)),0,.45,(180,180,180),1)
            marker_polygons=[]
            try:
                dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
                if hasattr(cv2.aruco,'ArucoDetector'):
                    corners,ids,_=cv2.aruco.ArucoDetector(dictionary,cv2.aruco.DetectorParameters()).detectMarkers(image)
                else:corners,ids,_=cv2.aruco.detectMarkers(image,dictionary)
                marker_polygons=[c.reshape(4,2).astype(np.float32) for c in corners]
                result['visible_aruco_ids']=[] if ids is None else [int(v) for v in ids.flatten()]
            except (cv2.error,AttributeError) as exc:result['aruco_exclusion_error']=str(exc)
            black.sort(key=lambda d:(d['bbox'][0],d['bbox'][1]))
            candidate_rows=[]
            locked_match=None
            if capture is not None:
                matches=[d for d in black if bbox_iou(capture['measurement_bbox'],d['bbox'])>=.5]
                if matches:locked_match=max(matches,key=lambda d:bbox_iou(capture['measurement_bbox'],d['bbox']))
            for number,d in enumerate(black,1):
                x,y,w,h=[float(v) for v in d['bbox']]
                locked=capture is not None and d is locked_match
                measurement_bbox=(capture['measurement_bbox'][:] if locked else d['bbox'][:])
                item=dict(detection_id=number,detection=d,geometry_candidate_available=False,
                    measurement_bbox=measurement_bbox,measurement_bbox_locked=locked,
                    raw_detector_bbox=d['bbox'][:],
                    geometry_input_policy='fixed_burst_bbox' if locked else 'live_detector_bbox')
                center=(x+w/2,y+h/2)
                if any(cv2.pointPolygonTest(p,center,False)>=0 for p in marker_polygons):
                    item['geometry_error']='ARUCO BOARD EXCLUDED'
                    candidate_rows.append(item)
                    continue
                try:
                    geometry=locate(image,measurement_bbox,k,model['diameter_mm'],model['total_height_mm'])
                    item['geometry']=geometry
                    if geometry['geometry_candidate_available']:
                        chosen=geometry['selected_candidate'];world={}
                        for field in ('top_center','geometric_body_center'):
                            p=np.asarray(chosen[field+'_camera_m'])
                            q=transform[:3,:3]@p+transform[:3,3]
                            inside=all(sum(e[j]*p[j] for j in range(3))+e[3]<=.005 for e in candidate['fit_camera_hull_equations'])
                            world[field+'_world_m']=q.tolist();world[field+'_inside_FIT_support']=inside
                            world[field+'_maximum_FIT_face_violation_mm']=float(max(
                                (np.dot(e[:3],p)+e[3])/np.linalg.norm(e[:3])*1000
                                for e in candidate['fit_camera_hull_equations']))
                        world['axis_from_top_into_body_world']=(transform[:3,:3]@np.asarray(chosen['axis_from_top_into_body_camera'])).tolist()
                        item.update(world,geometry_candidate_available=True)
                    else:item['geometry_error']=geometry['reject_reason']
                except (ValueError,RuntimeError,cv2.error,np.linalg.LinAlgError,FloatingPointError,OverflowError) as exc:
                    item['geometry_error']=str(exc)
                candidate_rows.append(item)
            click=selection['click'];selection['click']=None
            if click is not None and capture is None:
                choices=[r for r in candidate_rows if r.get('geometry_error')!='ARUCO BOARD EXCLUDED'
                    and r['detection']['bbox'][0]<=click[0]<=sum(r['detection']['bbox'][::2])
                    and r['detection']['bbox'][1]<=click[1]<=sum(r['detection']['bbox'][1::2])]
                if choices:selection['bbox']=min(choices,key=lambda r:r['detection']['bbox'][2]*r['detection']['bbox'][3])['detection']['bbox'][:]
            selected=None
            if selection['bbox'] is not None:
                choices=[r for r in candidate_rows if r.get('geometry_error')!='ARUCO BOARD EXCLUDED'
                    and bbox_iou(selection['bbox'],r['detection']['bbox'])>=.25]
                if choices:
                    selected=max(choices,key=lambda r:bbox_iou(selection['bbox'],r['detection']['bbox']))
                    selection['bbox']=selected['detection']['bbox'][:]
                else:result['geometry_error']='SELECTED TARGET LOST - CLICK AGAIN OR A FOR AUTO'
            else:
                qualified=[r for r in candidate_rows if r['geometry_candidate_available']]
                if len(qualified)==1:selected=qualified[0]
                elif len(qualified)>1:result['geometry_error']='MULTIPLE GEOMETRY CANDIDATES - CLICK MEASURED SHELL'
                else:
                    choices=[r for r in candidate_rows if r.get('geometry_error')!='ARUCO BOARD EXCLUDED']
                    if len(choices)==1:selected=choices[0]
                    else:result['geometry_error']='NO QUALIFIED GEOMETRY - CLICK MEASURED SHELL; S SAVES ALL REASONS'
            result['candidate_rows']=candidate_rows
            if selected is not None:
                result['selected_detection_id']=selected['detection_id']
                for name,value in selected.items():
                    if name not in ('detection','detection_id'):result[name]=value
            result['selection_mode']='operator_image_selection' if selection['bbox'] is not None else 'automatic_unique_geometry'
            if capture is not None and not result.get('measurement_bbox_locked',False):
                result['geometry_candidate_available']=False
                result['geometry_error']='FIXED ROI TARGET LOST OR IDENTITY CHANGED; original0.5 overlap required'
            if capture is not None:
                result['burst_locked_measurement_bbox']=capture['measurement_bbox'][:]
            if result['geometry_candidate_available']:
                try:result['equal_height_plane_projection']=project_observation(result,plane)
                except (ValueError,KeyError,np.linalg.LinAlgError,FloatingPointError,OverflowError) as exc:
                    result['equal_height_plane_error']=str(exc)
            completed=None;raw_completed=None
            if capture is not None:
                image_name='burst_%03d_frame_%03d.png'%(saved+1,len(capture['frames'])+1)
                image_path=folder/image_name
                if not cv2.imwrite(str(image_path),image,[cv2.IMWRITE_PNG_COMPRESSION,1]):
                    raise RuntimeError('Per-frame RGB evidence save failed')
                result['burst_RGB_file']=image_name
                result['burst_RGB_sha256']=sha(image_path)
                raw_completed=burst.observe(capture,result,time.monotonic())
                if raw_completed is not None:
                    completed=plane_summary(capture['frames'],raw_completed['elapsed_seconds'])
                    report['last_original_free3D_burst_summary']=raw_completed
                    report.setdefault('original_free3D_burst_results',[]).append(raw_completed)
                    saved+=1;stem='observation_%03d'%saved
                    payload=dict(result,burst_summary=completed,original_free3D_burst_summary=raw_completed,burst_frames=capture['frames'],
                        locked_measurement_bbox=capture['measurement_bbox'][:],
                        detector_bbox_is_not_3D_motion=True,per_frame_coordinates_smoothed=False)
                    last_capture_pass=completed['burst_qualified']
                    if last_capture_pass:
                        last_capture_message='VISUAL PICK TARGET SAVED; original free3D burst: '+('PASS' if raw_completed['burst_qualified'] else 'FAIL (retained)')
                    else:
                        top=completed.get('top_center_repeatability_max_mm')
                        body=completed.get('geometric_body_center_repeatability_max_mm')
                        if top is not None and body is not None and (top>3 or body>3):
                            last_capture_message='PLANE REJECTED: jitter top %.3f / body %.3f mm; limit3mm'%(top,body)
                        else:last_capture_message='PLANE REJECTED: '+','.join(completed['reject_reasons'])
                    report['last_capture_message']=last_capture_message
                    report['last_capture_pass']=last_capture_pass
                    if completed['burst_qualified']:
                        points.append(dict(observation=stem,**completed))
                        report['qualified_point_captures']=points
                        target_count+=1
                        reference=dict(schema_version=1,scope='live_visual_pick_target_point_only',
                            top_center_world_m=completed['top_center_world_m'][:],
                            burst_summary=completed,original_free3D_burst_summary=raw_completed,
                            equal_height_plane=plane,binding_camera_context=context,observation=stem,
                            observation_file=stem+'.json',reference_index=target_count,
                            historical_geometry_anchor_sha256=sha(folder/'geometry_source_anchor.json'),
                            old_grasp_tcp_mapping_used=False,old_2p8_plus_4p5_offsets_used=False,
                            wrist_pose_not_inferred_from_image=True,motion_authorized=False,execution_ready=False)
                        dump(folder/('reference_point_%03d.json'%target_count),reference)
                        dump(folder/'latest_reference_point.json',reference)
                        payload.update(binding_role='new_wrist_reference_visual_point',reference_point=reference)
                        report['latest_reference_point']=reference
                    dump(folder/(stem+'.json'),payload)
                    if not cv2.imwrite(str(folder/(stem+'.png')),image):raise RuntimeError('Raw image evidence save failed')
                    dump(folder/'capture_state.json',dict(report,saved_observations=saved,reference_points=target_count))
                    print('SAVED',stem,'PLANE_BURST_QUALIFIED:',completed['burst_qualified'],'ORIGINAL_FREE3D_BURST:',raw_completed['burst_qualified'],
                        'REASONS:',completed['reject_reasons'],flush=True)
                    if payload.get('reference_point') is not None:
                        print('VISUAL PICK TARGET SAVED; shell top World(mm):',
                            [v*1000 for v in completed['top_center_world_m']],
                            'OLD TCP MAPPING USED: False; NO MOTION',flush=True)
                    capture=None
            for item in candidate_rows:
                x,y,w,h=[int(v) for v in item['detection']['bbox']]
                active=selected is item
                color_box=(0,255,0) if active else (0,180,255)
                cv2.rectangle(view,(x,y),(x+w,y+h),color_box,2)
                label='#%d %s'%(item['detection_id'],'SELECTED' if active else (item.get('geometry_error') or 'GEOMETRY'))
                cv2.putText(view,label[:75],(max(0,x),max(15,y-20)),0,.45,color_box,1)
                if item.get('measurement_bbox_locked'):
                    mx,my,mw,mh=[int(v) for v in item['measurement_bbox']]
                    cv2.rectangle(view,(mx,my),(mx+mw,my+mh),(255,180,0),1)
                if item.get('geometry'):
                    ellipse=item['geometry']['ellipse']
                    cv2.ellipse(view,(tuple(ellipse['center_px']),tuple(ellipse['diameters_px']),ellipse['angle_deg']),(255,0,255),2)
            message=result.get('equal_height_plane_error') or result.get('geometry_error') or 'PLANE CANDIDATE - FIELD ACCURACY UNACCEPTED'
            if result['geometry_candidate_available'] and not result.get('equal_height_plane_error'):
                message='Raw free3D candidate; PLANE POINT REQUIRED'
            if result.get('equal_height_plane_projection') is not None:
                point=result['equal_height_plane_projection']['top_center_world_m']
                message='SAME SEATED SHELL TOP World mm: %.1f %.1f %.1f'%tuple(v*1000 for v in point)
            message_color=(0,255,0) if result['geometry_candidate_available'] else (0,140,255)
            if last_capture_message is not None and capture is None:
                message=last_capture_message
                message_color=(0,255,0) if last_capture_pass else (0,80,255)
            view=cv2.copyMakeBorder(view,100,0,0,0,cv2.BORDER_CONSTANT,value=(0,0,0))
            phase='VISUAL PICK TARGET | S SAME SEATED SHELL | saved: %d'%target_count
            if capture is not None:phase='CAPTURING FIXED ROI - KEEP SHELL STILL (%d frames)'%len(capture['frames'])
            cv2.putText(view,phase+' | Click / 1-9 select | NO ROBOT COMMANDS',(12,28),0,.60,(0,255,255),2)
            cv2.putText(view,message[:120],(12,60),0,.55,message_color,1)
            outline=result.get('geometry',{}).get('contour_detail',{})
            mode='RAY + EQUAL HEIGHT PLANE; height inferred'
            if result['geometry_candidate_available'] and not result.get('top_center_inside_FIT_support',False):
                mode+=' | OUTSIDE FIT'
            cv2.putText(view,mode+' | EQUAL HEIGHT assumption; normal estimated; NO MOTION / no free3D pass',(12,85),0,.5,(180,180,255),1)
            if completed is not None and not cv2.imwrite(str(folder/(stem+'_view.png')),view):
                raise RuntimeError('Display evidence save failed')
            cv2.imshow(title,view);key=cv2.waitKey(1)&255
            if (args.finish_after_first_reference and completed is not None
                    and completed['burst_qualified']):break
            if key in (27,ord('q'),ord('Q')) or cv2.getWindowProperty(title,cv2.WND_PROP_VISIBLE)<1:break
            if capture is None and key in (ord('a'),ord('A')):selection['bbox']=None
            if capture is None and ord('1')<=key<=ord('9'):
                chosen=[r for r in candidate_rows if r['detection_id']==key-ord('0') and r.get('geometry_error')!='ARUCO BOARD EXCLUDED']
                if chosen:selection['bbox']=chosen[0]['detection']['bbox'][:]
            if key in (ord('s'),ord('S')):
                if capture is not None:print('正在采集，请保持空壳不动；Q可保存中断记录并退出。',flush=True)
                elif not result['geometry_candidate_available'] or result.get('equal_height_plane_projection') is None:
                    saved+=1;stem='observation_%03d'%saved
                    dump(folder/(stem+'.json'),result)
                    if not cv2.imwrite(str(folder/(stem+'.png')),image) or not cv2.imwrite(str(folder/(stem+'_view.png')),view):
                        raise RuntimeError('Rejected image evidence save failed')
                    last_capture_pass=False
                    last_capture_message='PLANE REJECTED: '+str(result.get('equal_height_plane_error') or result.get('geometry_error'))
                    report['last_capture_message']=last_capture_message;report['last_capture_pass']=False
                    print('SAVED REJECTED',stem,result.get('equal_height_plane_error') or result.get('geometry_error'),flush=True)
                else:
                    selection['bbox']=next(r['detection']['bbox'][:] for r in candidate_rows if r['detection_id']==result['selected_detection_id'])
                    capture=burst.begin(time.monotonic())
                    capture['measurement_bbox']=selection['bbox'][:]
                    capture['measurement_bbox_origin']='selected detector bbox at this S; immutable until burst completes'
                    last_capture_message=None;last_capture_pass=None
                    report['last_locked_measurement_bbox']=capture['measurement_bbox'][:]
                    print('CAPTURING：保持空壳静止，至少8帧/约2秒；采样窗口上限15秒，不含单帧推理/取帧阻塞。',flush=True)
        report['outcome']=('live_visual_pick_target_point_saved' if target_count else
            'no_qualified_live_pick_target')
        report['reference_points']=target_count
        report['six_dimensional_TCP_calibrated']=False
        report['motion_authorized']=False
        return 0
    except BaseException as exc:
        report.update(error=repr(exc),traceback=traceback.format_exc(),outcome='failed_no_robot_access')
        print('ERROR:',exc,flush=True)
        return 1
    finally:
        cleanup_errors=[]
        try:
            if started:pipeline.stop()
        except Exception as exc:cleanup_errors.append('pipeline.stop: '+repr(exc))
        try:
            if client is not None:client.close()
        except Exception as exc:cleanup_errors.append('client.close: '+repr(exc))
        report['saved_observations']=saved
        if capture is not None:
            try:
                dump(folder/'interrupted_burst.json',dict(capture,interrupted=True,execution_ready=False))
            except Exception as exc:cleanup_errors.append('interrupted record: '+repr(exc))
        report['cleanup_errors']=cleanup_errors
        try:
            dump(folder/'result.json',report)
        except Exception as exc:
            cleanup_errors.append('result record: '+repr(exc))
            try:
                dump(folder/'result_fallback.json',dict(outcome='record_write_failed',
                    original_error=str(report.get('error')),cleanup_errors=cleanup_errors,execution_ready=False))
            except Exception as nested:print('RECORD FAILED:',repr(nested),flush=True)
        # Packaging is attempted even when another record or cleanup operation failed.
        try:
            archive=archive_folder(folder)
            print('SAVED:',archive,'EXECUTION_READY: False',flush=True)
        except Exception as exc:print('ARCHIVE FAILED:',repr(exc),'FILES RETAINED:',folder,flush=True)


if __name__=='__main__':sys.exit(main())
