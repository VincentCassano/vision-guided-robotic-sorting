"""Color-aware photometric silhouette, with unchanged circle-pose geometry.

Black delegates byte-for-byte to its accepted pipeline. Colored shells use one
observed outer silhouette and the same upright axis as support inference.
Detector boxes select the ROI; measured foreground bounds score body geometry.
"""
import cv2
import numpy as np
from scipy.optimize import minimize_scalar,least_squares
import known_cylinder_geometry_axis as base


def known_axis_upper_silhouette_ellipse(gray, bbox_local, offset, upright_vanishing):
    """Measure the observed top outline in the known upright image-axis basis.

    The existing fixed upright support axis supplies image orientation.
    PCA elongation is recorded rather than used to infer this known axis.
    The real mask pixels and original pixel-quality constraints are retained.
    Every selected upper-profile point still participates in the unchanged2px
    fit gate. No residual-based point deletion or new world-coordinate fit.
    """
    x,y,w,h=bbox_local
    _,dark=cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
    count,labels,stats,_=cv2.connectedComponentsWithStats(dark,connectivity=8)
    best=None
    for index in range(1,count):
        left,top,cw,ch,area=[int(v) for v in stats[index]]
        intersection=max(0,min(left+cw,x+w)-max(left,x))*max(0,min(top+ch,y+h)-max(top,y))
        if area<.2*w*h or intersection<.4*w*h:continue
        if best is None or intersection>best[0]:best=intersection,index
    if best is None:raise ValueError('No isolated dark cylinder silhouette in expanded ROI')
    ys,xs=np.where(labels==best[1])
    if min(xs)==0 or min(ys)==0 or max(xs)==gray.shape[1]-1 or max(ys)==gray.shape[0]-1:
        raise ValueError('Dark object leaves expanded ROI/image: partial object, not a complete shell')
    pixels=np.column_stack((xs,ys)).astype(float)
    origin=np.mean(pixels,axis=0)
    eigenvalues,eigenvectors=np.linalg.eigh(np.cov((pixels-origin).T))
    if not np.isfinite(eigenvalues).all() or eigenvalues[0]<=0:
        raise ValueError('Observed body pixels are degenerate')
    vanishing=np.asarray(upright_vanishing,float)
    if vanishing.shape!=(3,) or not np.isfinite(vanishing).all():
        raise ValueError('Existing upright-axis image projection is malformed')
    # d(pixel)/d(length) has this direction at the actual mask centroid;
    # positive camera depth cancels. No bbox center/depth becomes a grasp.
    image_origin=origin+offset
    down=vanishing[:2]-image_origin*vanishing[2]
    norm=float(np.linalg.norm(down))
    if not np.isfinite(norm) or norm<1e-9:
        raise ValueError('Upright image direction is singular at this object')
    down=down/norm
    if down[1]<=0:raise ValueError('Cannot establish downward body axis from image')
    across=np.array([down[1],-down[0]])
    basis=np.column_stack((across,down))
    aligned=(pixels-origin)@basis
    body_width=float(np.ptp(aligned[:,0]));body_length=float(np.ptp(aligned[:,1]))
    top=float(np.min(aligned[:,1]));stop=top+.35*body_length
    upper_points=aligned[aligned[:,1]<stop]
    if not len(upper_points):raise ValueError('No axis-aligned upper silhouette')
    # One-pixel bins in an orthonormal basis keep the original pixel unit.
    columns=np.rint(upper_points[:,0]).astype(int)
    left,right=int(min(columns)),int(max(columns));span=right-left
    if not .65*body_width<=span<=1.15*body_width:
        raise ValueError('Axis-aligned top width disagrees with observed body cross-width')
    points=[]
    for column in range(left,right+1):
        group=upper_points[columns==column]
        if len(group):
            observed=group[np.argmin(group[:,1])]
            points.append(observed.tolist())
    points=np.asarray(points,float)
    if len(points)<20 or np.ptp(points[:,0])<.85*span:
        raise ValueError('Insufficient observed axis-aligned top arc')
    side_count=max(2,len(points)//10)
    shoulder=float(np.median(np.r_[points[:side_count,1],points[-side_count:,1]]))
    sag=shoulder-float(np.median(points[len(points)//2-side_count:len(points)//2+side_count,1]))
    if sag<4:raise ValueError('Upper outline is too flat to establish an ellipse; not treating a board as a cap')
    center_x=.5*(left+right)
    initial=[center_x,shoulder,span/2+.5,max(4.1,min(sag,.59*span)),0.]
    lower=[center_x-2,top,span/2-1,4,-.7]
    upper=[center_x+2,top+.40*body_length,span/2+3,.60*span,.7]
    if any(a>=b for a,b in zip(lower,upper)):raise ValueError('Top arc bounds invalid at this image scale')
    initial=np.clip(initial,np.asarray(lower)+1e-5,np.asarray(upper)-1e-5)

    def residual(parameters):
        cx,cy,a,b,slope=parameters
        dx=points[:,0]-cx
        inside=1-(dx/a)**2
        predicted=cy+slope*dx-b*np.sqrt(np.maximum(inside,1e-9))
        return np.r_[predicted-points[:,1],span*np.minimum(inside,0)]

    solved=least_squares(residual,initial,bounds=(lower,upper),loss='soft_l1',f_scale=1.,max_nfev=80)
    if not solved.success:raise ValueError('Observed axis-aligned upper arc fit did not converge')
    error=float(np.percentile(np.abs(residual(solved.x)[:len(points)]),90))
    if error>2:
        raise ValueError('Axis-aligned upper arc residual %.2fpx exceeds original2px image-quality gate'%error)
    cx,cy,a,b,slope=solved.x
    local_shape=np.array([[1/a**2+slope**2/b**2,-slope/b**2],[-slope/b**2,1/b**2]])
    image_shape=basis@local_shape@basis.T
    values,vectors=np.linalg.eigh(image_shape)
    if not np.isfinite(values).all() or min(values)<=0:raise ValueError('Invalid fitted cap shape')
    diameters=2/np.sqrt(values)
    angle=float(np.degrees(np.arctan2(vectors[1,0],vectors[0,0])))
    center=origin+basis@np.array([cx,cy])+offset
    contour=points@basis.T+origin+offset
    # Additional old closed-contour radial pixel gate; never a substitute for
    # the upper-profile2px gate above.
    local_delta=points-[cx,cy]
    radial=np.sqrt(np.einsum('ij,ij->i',local_delta@local_shape,local_delta))
    radial_error=float(np.percentile(np.abs(radial-1)*min(diameters)/2,90))
    if radial_error>2:raise ValueError('Axis-aligned cap radial residual exceeds original2px gate')
    ellipse=(tuple(float(v) for v in center),tuple(float(v) for v in diameters),angle)
    return ellipse,dict(contour_error_p90_px=error,radial_error_p90_px=radial_error,
        contour_px=contour.tolist(),selection='known_upright_axis_outer_upper_silhouette',
        image_axis_source='same_fixed_upright_axis_as_two_support_plane_model',
        upright_vanishing_homogeneous=vanishing.tolist(),
        PCA_variance_ratio_is_diagnostic=True,
        original_PCA_ratio_below2=bool(eigenvalues[1]/eigenvalues[0]<2),
        observed_outline='upper_arc_only',lower_half_inferred=True,
        uncertainty_scope='known_upright_axis_outline_not_independently_position_accepted',
        observed_sag_px=sag,fitted_sag_px=float(b),image_body_axis=down.tolist(),
        principal_variance_ratio=float(eigenvalues[1]/eigenvalues[0]),
        body_aligned_width_px=body_width,body_aligned_length_px=body_length,
        body_measurement_bbox_px=[float(min(xs)+offset[0]),float(min(ys)+offset[1]),
            float(max(xs)-min(xs)),float(max(ys)-min(ys))],
        body_measurement_source='actual_selected_foreground_component_pixel_bounds',
        detector_bbox_width_height_px=[float(w),float(h)],profile_points=len(points),
        residual_based_point_deletion=False,pixel_fit_gate=2.,world_transform_changed=False)



def color_cap(frame,bbox,class_id,upright_vanishing=None):
    # Fix the measured physical feature: do not alternate a closed internal
    # edge and the exterior upper silhouette between frames. The known axis
    # is shared with support inference, not inferred from detector-box shape.
    if upright_vanishing is None:
        raise ValueError('Colored outline requires the existing upright model image direction')
    axis_policy='colored_axis_from_existing_upright_support_model_not_mask_PCA'
    x,y,w,h=[float(v) for v in bbox];height,width=frame.shape[:2]
    if min(w,h)<25 or x<2 or y<2 or x+w>width-2 or y+h>height-2:
        raise ValueError('Colored shell is too small or clipped by the image border')
    pad=max(12,int(np.ceil(.20*w)))
    x0,y0=max(0,int(x)-pad),max(0,int(y)-pad)
    x1,y1=min(width,int(np.ceil(x+w))+pad),min(height,int(np.ceil(y+h))+pad)
    roi=frame[y0:y1,x0:x1]
    left,top=int(x)-x0,int(y)-y0;right,bottom=int(np.ceil(x+w))-x0,int(np.ceil(y+h))-y0
    # The detector rectangle is only an initializer. Its surrounding band
    # stays probable background, allowing the real rim to extend beyond a
    # tight YOLO box instead of creating an artificial box-clipped contour.
    labels=np.full(roi.shape[:2],cv2.GC_PR_BGD,np.uint8)
    labels[[0,-1],:]=cv2.GC_BGD;labels[:,[0,-1]]=cv2.GC_BGD
    labels[top:bottom,left:right]=cv2.GC_PR_FGD
    # Only a small central body patch seeds foreground. The output contour
    # remains measured from real RGB foreground/background evidence.
    ax,bx=int(left+.40*w),int(left+.60*w)
    ay,by=int(top+.40*h),int(top+.60*h)
    labels[ay:by,ax:bx]=cv2.GC_FGD
    if class_id==1:
        hsv=cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
        red=((hsv[:,:,0]<=15)|(hsv[:,:,0]>=165))&(hsv[:,:,1]>=80)&(hsv[:,:,2]>=30)
        inside=np.zeros(labels.shape,bool);inside[top:bottom,left:right]=True
        labels[red&inside]=cv2.GC_FGD
    background=np.zeros((1,65),np.float64);foreground=np.zeros((1,65),np.float64)
    cv2.setRNGSeed(2303)
    cv2.grabCut(roi,labels,None,background,foreground,5,cv2.GC_INIT_WITH_MASK)
    mask=(labels==cv2.GC_FGD)|(labels==cv2.GC_PR_FGD)
    if not np.any(mask):raise ValueError('No isolated colored-shell foreground')
    if np.any(mask[1,:]) or np.any(mask[-2,:]) or np.any(mask[:,1]) or np.any(mask[:,-2]):
        raise ValueError('Colored foreground reaches expanded ROI; complete rim unavailable')
    transitions=np.diff(np.r_[0,mask.ravel().astype(np.int16),0])
    starts=np.flatnonzero(transitions==1);ends=np.flatnonzero(transitions==-1)
    # Reuse the observed silhouette, fixed upright axis and original2px
    # gate. Binary intensity encodes segmentation, never a fabricated ellipse.
    segmented=np.where(mask,0,255).astype(np.uint8)
    ellipse,detail=known_axis_upper_silhouette_ellipse(segmented,
        (x-x0,y-y0,w,h),np.array([x0,y0]),upright_vanishing)
    detail=dict(detail,color_class_id=class_id,color_preprocessing='original_RGB_GrabCut_foreground',
        original_edge_failure=None,axis_policy=axis_policy,
        white_axis_policy=axis_policy if class_id==2 else None,segmentation_bbox_px=list(bbox),
        segmentation_rng_seed=2303,outer_feature_policy='same_exterior_upper_profile_every_frame',
        foreground_pixels=int(mask.sum()),RGB_pixels_and_pixel_coordinates_preserved=True,
        foreground_mask_shape=list(mask.shape),expanded_ROI_origin_px=[x0,y0],
        foreground_mask_RLE=[[int(a),int(b-a)] for a,b in zip(starts,ends)],
        bbox_used_for_segmentation_only=True,contour_synthesized_from_bbox=False,
        morphology_or_coordinate_smoothing=False,new_color_geometry_field_accepted=False)
    return ellipse,detail


def locate(frame,bbox,intrinsics,diameter_mm,total_height_mm,class_id,upright_vanishing=None):
    if class_id==0:return base.locate(frame,bbox,intrinsics,diameter_mm,total_height_mm)
    if class_id not in (1,2):raise ValueError('Unsupported noncylinder target')
    k=np.asarray(intrinsics,float);radius=float(diameter_mm)/2000.;height=float(total_height_mm)/1000.
    if not 0<radius<height or k.shape!=(3,3):raise ValueError('Known cylinder dimensions and K required')
    ellipse,detail=color_cap(frame,bbox,class_id,upright_vanishing)
    roots=base.circle_pose_candidates(ellipse,k,radius)
    measured=detail['body_measurement_bbox_px']
    x,y,w,h=[float(v) for v in measured];observed_bbox=np.array([x,y,x+w,y+h]);choices=[]
    for root in roots:
        def evaluate(visible_height):
            cloud=np.vstack([root['rim'],root['rim']+visible_height*root['normal']])
            if np.any(cloud[:,2]<=0):return 1e6
            pixels=cloud@k.T;pixels=pixels[:,:2]/pixels[:,2:3]
            box=np.r_[pixels.min(axis=0),pixels.max(axis=0)]
            return float(np.sqrt(np.mean((box-observed_bbox)**2)))
        fitted=minimize_scalar(evaluate,bounds=(.005,height),method='bounded',options={'maxiter':40})
        choices.append(dict(top_center_camera_m=root['point'].tolist(),
            axis_from_top_into_body_camera=root['normal'].tolist(),
            geometric_body_center_camera_m=(root['point']+.5*height*root['normal']).tolist(),
            visible_height_estimated_from_silhouette_mm=float(fitted.x*1000),
            body_bbox_rms_px=float(fitted.fun),circle_reprojection_p95_px=root['circle_reprojection_p95_px']))
    choices.sort(key=lambda r:r['body_bbox_rms_px']);best=choices[0]
    maximum=max(3.,.03*max(w,h));quality=best['body_bbox_rms_px']<=maximum;ambiguous=False
    if len(choices)>1:
        a=np.array(best['axis_from_top_into_body_camera']);b=np.array(choices[1]['axis_from_top_into_body_camera'])
        ambiguous=a@b<.999 and choices[1]['body_bbox_rms_px']-best['body_bbox_rms_px']<2
    result=dict(method='color_photometric_outline_same_measured_circle_geometry_v23',
        ellipse=dict(center_px=list(ellipse[0]),diameters_px=list(ellipse[1]),angle_deg=ellipse[2]),
        contour_detail=detail,pose_branches=choices,geometry_candidate_available=quality and not ambiguous,
        reject_reason='ambiguous_circle_pose' if ambiguous else (None if quality else 'body_silhouette_mismatch'),
        diameter_mm=diameter_mm,total_height_mm=total_height_mm,maximum_body_bbox_rms_px=maximum,
        body_bbox_measurement_source='observed_foreground_bounds_not_YOLO_bbox',
        observed_body_bbox_px=measured[:],detector_bbox_px=list(bbox),
        independent_object_position_accepted=False,per_slot_lookup=False,RGBD_depth_used=False,execution_ready=False)
    if result['geometry_candidate_available']:result['selected_candidate']=best
    return result
