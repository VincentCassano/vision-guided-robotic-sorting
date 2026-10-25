"""Live YOLO + measured-cylinder RGB geometry candidates; no robot SDK or commands."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys
import time
import traceback
from known_cylinder_records import archive_folder, dump, encode

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
    parser.add_argument('--reference-spacing-mm',type=float,
        help='Optional measured center-to-center distance of two equal-height full recesses; diagnostic only')
    args=parser.parse_args()
    if args.reference_spacing_mm is not None and (not 0<args.reference_spacing_mm<1000):
        parser.error('Reference spacing must be finite and between 0 and 1000 mm')
    folder=ROOT/'captures'/('known_cylinder_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True,exist_ok=False)
    pipeline=None;client=None;started=False;saved=0;capture=None;points=[]
    report=dict(scope='live_known_measured_black_shell_geometry_development',
        delivery_version='known-cylinder-vision-v4-20261020',
        robot_SDK_loaded=False,arm_connections=0,arm_writes=0,hand_writes=0,
        controller_configuration_writes=0,camera_option_writes=0,execution_ready=False,
        full6D_handeye_pass=False,RGBD_position_validation_pass=False,
        object_geometry_independently_accepted=False,source_session=str(args.position_session))
    try:
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
        from known_cylinder_geometry import locate
        import known_cylinder_capture as burst
        thresholds=yaml.safe_load((ROOT/'src/battery_grasp/config/yolo_v3_calibrated.yaml').read_text())['battery_detector_yolo']['class_confidences']
        report.update(candidate_sha256=candidate_sha,marker_position_qualification=qualification,
            measured_shell=model,weights_sha256=sha(weights),class_confidences=thresholds,
            reference_spacing_mm=args.reference_spacing_mm,
            runtime_source_sha256={name:sha(ROOT/'deployment'/name) for name in
                ('run_known_cylinder_vision.py','known_cylinder_geometry.py','known_cylinder_capture.py',
                 'known_cylinder_records.py')})
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
        title='Known black shell | YOLO + RGB geometry | S save Q finish'
        cv2.namedWindow(title,cv2.WINDOW_NORMAL)
        selection=dict(bbox=None,click=None)
        def mouse(event,x,y,flags,param):
            if event==cv2.EVENT_LBUTTONDOWN and y>=100:selection['click']=(x,y-100)
        cv2.setMouseCallback(title,mouse)
        print('仅保留本次量过的一个黑色空壳：直径45.8mm、高98mm；印刷板可保留在臂上，头部不动。',flush=True)
        print('自动运行现有YOLO与圆柱图像几何；不启动深度流、不连接机器人、不移动。S保存当前实际结果，Q统一打包。',flush=True)
        print('新物体几何方法未实物验收，候选坐标不能作为运动指令；失败也保存原因/原图，不要求重采16组。',flush=True)
        print('v4连续采集：点真实空壳或按其编号，S开始约2秒采集，采集时不要动空壳；A SAVED也在图像顶部显示。Q统一打包。',flush=True)
        if args.reference_spacing_mm is not None:
            print('先保持空壳在当前完整槽，选中后按S采A；看到A SAVED后，把同一空壳移入相距%.1fmm的同侧等高完整槽，落底、撤手，再选中按S采B。不要移臂/头/板。'%args.reference_spacing_mm,flush=True)
        while True:
            frames=pipeline.wait_for_frames(5000);color=frames.get_color_frame()
            if not color:continue
            image=np.asanyarray(color.get_data()).copy()
            detections=client.detect(image)
            black=[d for d in detections if d['class_id']==0 and d['confidence']>=thresholds[0]]
            result=dict(delivery_version=report['delivery_version'],
                frame_number=int(color.get_frame_number()),timestamp_ms=float(color.get_timestamp()),
                detections=detections,geometry_candidate_available=False,execution_ready=False,
                selection_scope='live_image_detection_only_not_taught_grasp_coordinates')
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
            for number,d in enumerate(black,1):
                x,y,w,h=[float(v) for v in d['bbox']]
                item=dict(detection_id=number,detection=d,geometry_candidate_available=False)
                center=(x+w/2,y+h/2)
                if any(cv2.pointPolygonTest(p,center,False)>=0 for p in marker_polygons):
                    item['geometry_error']='ARUCO BOARD EXCLUDED'
                    candidate_rows.append(item)
                    continue
                try:
                    geometry=locate(image,d['bbox'],k,model['diameter_mm'],model['total_height_mm'])
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
            completed=None
            if capture is not None:
                completed=burst.observe(capture,result,time.monotonic())
                if completed is not None:
                    saved+=1;stem='observation_%03d'%saved
                    payload=dict(result,burst_summary=completed,burst_frames=capture['frames'])
                    if completed['burst_qualified']:
                        points.append(dict(observation=stem,**completed))
                        report['qualified_point_captures']=points
                        if args.reference_spacing_mm is not None and len(points)==2:
                            report['relative_pair_result']=burst.relative_pair(points[0],points[1],args.reference_spacing_mm)
                            payload['relative_pair_result']=report['relative_pair_result']
                    dump(folder/(stem+'.json'),payload)
                    if not cv2.imwrite(str(folder/(stem+'.png')),image):raise RuntimeError('Raw image evidence save failed')
                    dump(folder/'capture_state.json',dict(report, saved_observations=saved))
                    if payload.get('relative_pair_result') is not None:
                        print('RELATIVE PAIR:',encode(payload['relative_pair_result']),flush=True)
                    print('SAVED',stem,'BURST_QUALIFIED:',completed['burst_qualified'],
                        'REASONS:',completed['reject_reasons'],flush=True)
                    if args.reference_spacing_mm is not None and len(points)==1 and completed['burst_qualified']:
                        print('A SAVED。现在将同一空壳搬到相距%.1fmm的同侧等高完整槽，完全落底并撤手；重新选真实空壳，按S采B。'%args.reference_spacing_mm,flush=True)
                        selection['bbox']=None
                    elif args.reference_spacing_mm is not None and len(points)==2:
                        print('A/B采集完成，按Q打包；相对检查不代表绝对精度通过，不开放运动。',flush=True)
                    capture=None
            for item in candidate_rows:
                x,y,w,h=[int(v) for v in item['detection']['bbox']]
                active=selected is item
                color_box=(0,255,0) if active else (0,180,255)
                cv2.rectangle(view,(x,y),(x+w,y+h),color_box,2)
                label='#%d %s'%(item['detection_id'],'SELECTED' if active else (item.get('geometry_error') or 'GEOMETRY'))
                cv2.putText(view,label[:75],(max(0,x),max(15,y-20)),0,.45,color_box,1)
                if item.get('geometry'):
                    ellipse=item['geometry']['ellipse']
                    cv2.ellipse(view,(tuple(ellipse['center_px']),tuple(ellipse['diameters_px']),ellipse['angle_deg']),(255,0,255),2)
            message=result.get('geometry_error') or 'GEOMETRY CANDIDATE - FIELD ACCURACY UNACCEPTED'
            if result['geometry_candidate_available']:
                message='BODY CENTER World mm: %.1f %.1f %.1f'%tuple(1000*np.asarray(result['geometric_body_center_world_m']))
            if report.get('relative_pair_result'):
                pair=report['relative_pair_result']
                message='RELATIVE %s | XY %.1f / ref %.1f mm | dZ %.1f mm | Q finish'%(
                    'PASS' if pair['relative_check_pass'] else 'FAIL',pair['observed_horizontal_distance_mm'],
                    pair['reference_spacing_mm'],pair['height_difference_mm'])
            view=cv2.copyMakeBorder(view,100,0,0,0,cv2.BORDER_CONSTANT,value=(0,0,0))
            phase='A: current recess' if not points else ('A SAVED - move shell %.1fmm; select, S for B'%args.reference_spacing_mm if args.reference_spacing_mm is not None and len(points)==1 else 'A/B SAVED - Q to finish')
            if args.reference_spacing_mm is None:phase='S: 2s live burst; Q finish'
            if capture is not None:phase='CAPTURING - DO NOT MOVE SHELL (%d frames)'%len(capture['frames'])
            cv2.putText(view,phase+' | Click / 1-9 select | NO ROBOT COMMANDS',(12,28),0,.60,(0,255,255),2)
            cv2.putText(view,message[:120],(12,60),0,.55,(0,255,0) if result['geometry_candidate_available'] else (0,140,255),1)
            outline=result.get('geometry',{}).get('contour_detail',{})
            mode='UPPER ARC; LOWER HALF INFERRED' if outline.get('lower_half_inferred') else 'Closed rim / no qualified rim'
            if result['geometry_candidate_available'] and not result.get('top_center_inside_FIT_support',False):
                mode+=' | OUTSIDE FIT'
            cv2.putText(view,mode+' | Candidate only; accuracy and grasp TCP pending',(12,85),0,.5,(180,180,255),1)
            if completed is not None and not cv2.imwrite(str(folder/(stem+'_view.png')),view):
                raise RuntimeError('Display evidence save failed')
            cv2.imshow(title,view);key=cv2.waitKey(1)&255
            if key in (27,ord('q'),ord('Q')) or cv2.getWindowProperty(title,cv2.WND_PROP_VISIBLE)<1:break
            if capture is None and key in (ord('a'),ord('A')):selection['bbox']=None
            if capture is None and ord('1')<=key<=ord('9'):
                chosen=[r for r in candidate_rows if r['detection_id']==key-ord('0') and r.get('geometry_error')!='ARUCO BOARD EXCLUDED']
                if chosen:selection['bbox']=chosen[0]['detection']['bbox'][:]
            if key in (ord('s'),ord('S')):
                if capture is not None:print('正在采集，请保持空壳不动；Q可保存中断记录并退出。',flush=True)
                elif args.reference_spacing_mm is not None and len(points)>=2:print('A/B已完成，请按Q统一打包。',flush=True)
                elif not result['geometry_candidate_available']:
                    saved+=1;stem='observation_%03d'%saved
                    dump(folder/(stem+'.json'),result)
                    if not cv2.imwrite(str(folder/(stem+'.png')),image) or not cv2.imwrite(str(folder/(stem+'_view.png')),view):
                        raise RuntimeError('Rejected image evidence save failed')
                    print('SAVED REJECTED',stem,result.get('geometry_error'),flush=True)
                else:
                    selection['bbox']=next(r['detection']['bbox'][:] for r in candidate_rows if r['detection_id']==result['selected_detection_id'])
                    capture=burst.begin(time.monotonic())
                    print('CAPTURING：保持空壳静止，至少8帧/约2秒；采样窗口上限15秒，不含单帧推理/取帧阻塞。',flush=True)
        if args.reference_spacing_mm is not None:
            pair=report.get('relative_pair_result')
            report['outcome']=('relative_pair_pass_absolute_accuracy_pending' if pair['relative_check_pass'] else
                'relative_pair_failed') if pair else 'incomplete_relative_pair'
        else:report['outcome']='live_geometry_observations_saved' if saved else 'no_observation_saved'
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
