"""Known-size cylinder RGB geometry with an image-axis top outline; no robot driver.

An observed cap ellipse constrains a circle of measured radius. Two circle-pose
branches are retained; the visible body silhouette must distinguish them. This
is a new perception method, not an acceptance certificate or motion permission.
"""
import cv2
import numpy as np
from scipy.optimize import least_squares, minimize_scalar


def ellipse_conic(ellipse):
    center, diameters, angle = ellipse
    center = np.asarray(center, float)
    axes = np.asarray(diameters, float)/2
    if not np.isfinite([*center, *axes, angle]).all() or min(axes) <= 0:
        raise ValueError('Invalid observed cap ellipse')
    theta = np.deg2rad(angle)
    rotation = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    shape = rotation@np.diag(1/axes**2)@rotation.T
    conic = np.zeros((3, 3))
    conic[:2, :2] = shape
    conic[:2, 2] = conic[2, :2] = -shape@center
    conic[2, 2] = center@shape@center-1
    return conic


def upper_silhouette_ellipse(gray, bbox_local, offset):
    """Measure the top outline in the observed body's principal-axis basis.

    The detector bbox bounds a possibly slanted whole body. Image-column tops
    may therefore include its side. An orthonormal image-plane rotation only
    changes the measurement basis; it is not a camera/world compensation.
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
    if (not np.isfinite(eigenvalues).all() or eigenvalues[0]<=0
            or eigenvalues[1]/eigenvalues[0]<2):
        raise ValueError('Body image axis ambiguous; principal variance ratio below new conservative2:1 gate')
    down=eigenvectors[:,1]
    if down[1]<0:down=-down
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
        contour_px=contour.tolist(),selection='axis_aligned_outer_upper_silhouette',
        observed_outline='upper_arc_only',lower_half_inferred=True,
        uncertainty_scope='new_axis_outline_method_not_independently_position_accepted',
        observed_sag_px=sag,fitted_sag_px=float(b),image_body_axis=down.tolist(),
        principal_variance_ratio=float(eigenvalues[1]/eigenvalues[0]),
        body_aligned_width_px=body_width,body_aligned_length_px=body_length,
        detector_bbox_width_height_px=[float(w),float(h)],profile_points=len(points),
        residual_based_point_deletion=False,pixel_fit_gate=2.,world_transform_changed=False)


def cap_ellipse(frame, bbox):
    x, y, w, h = [float(v) for v in bbox]
    height, width = frame.shape[:2]
    if min(w, h) < 25 or x < 2 or y < 2 or x+w > width-2 or y+h > height-2:
        raise ValueError('Object too small or clipped by image border')
    # Detector boxes often end exactly on the outer rim. Preserve a surrounding
    # background band instead of clipping the very edge being measured.
    pad = max(8, int(np.ceil(.15*w)))
    x0, y0 = max(0, int(x)-pad), max(0, int(y)-pad)
    x1, y1 = min(width, int(np.ceil(x+w))+pad), min(height, int(np.ceil(y+h))+pad)
    gray = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    edges = cv2.Canny(gray, 40, 120)
    contours = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)[-2]
    candidates = []
    for contour in contours:
        if len(contour) < 20:
            continue
        ellipse = cv2.fitEllipse(contour)
        (cx, cy), (a, b), angle = ellipse
        major, minor = max(a, b), min(a, b)
        local_x, local_y = cx-(x-x0), cy-(y-y0)
        if (not .65*w <= major <= 1.15*w or minor < 8 or minor/major < .15
                or not .15*w < local_x < .85*w or not -.05*h < local_y < .60*h):
            continue
        points = contour.reshape(-1, 2).astype(float)
        theta = np.deg2rad(angle)
        rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        normalized = ((points-[cx, cy])@rot)/np.array([a/2, b/2])
        radial_error = np.abs(np.linalg.norm(normalized, axis=1)-1)*minor/2
        sectors = np.floor((np.arctan2(normalized[:, 1], normalized[:, 0])+np.pi)*4/np.pi).astype(int)%8
        error = float(np.percentile(radial_error, 90))
        if error > 2 or len(set(sectors.tolist())) < 7:
            continue
        full = ((float(cx+x0), float(cy+y0)), (float(a), float(b)), float(angle))
        candidates.append((major*minor, full, error, points+np.array([x0, y0])))
    if not candidates:
        return upper_silhouette_ellipse(gray, (x-x0,y-y0,w,h), np.array([x0,y0]))
    # Prefer the outer rim by image area, never by a desirable 3D answer.
    _, ellipse, error, contour = max(candidates, key=lambda row: row[0])
    return ellipse, dict(contour_error_p90_px=error, contour_px=contour.tolist(),
                        selection='largest_qualified_upper_cap_ellipse')


def circle_pose_candidates(ellipse, k, radius_m):
    observed_pixel = ellipse_conic(ellipse)
    observed = k.T@observed_pixel@k
    observed /= np.linalg.norm(observed)
    if np.trace(observed) < 0:
        observed = -observed
    shape = observed[:2, :2]
    center = -np.linalg.solve(shape, observed[:2, 2])
    scale = center@shape@center-observed[2, 2]
    eigenvalues, eigenvectors = np.linalg.eigh(shape/scale)
    if min(eigenvalues) <= 0:
        raise ValueError('Observed rim is not a finite ellipse')
    radii = 1/np.sqrt(eigenvalues)
    major, minor = max(radii), min(radii)
    initial_z = float(np.clip(radius_m/major, .101, 1.999))
    sight = np.r_[center, 1.]
    sight /= np.linalg.norm(sight)
    small_axis = eigenvectors[:, np.argmax(eigenvalues)]
    tangent = np.r_[small_axis, 0.]
    tangent -= (tangent@sight)*sight
    tangent /= np.linalg.norm(tangent)
    cosine = float(np.clip(minor/major, .05, 1.))
    tilt = np.arccos(cosine)
    initials = [sight*np.cos(tilt)+sign*tangent*np.sin(tilt) for sign in (-1, 1)]
    initials.append(sight)
    indices = np.triu_indices(3)
    weights = np.where(indices[0] == indices[1], 1., np.sqrt(2.))

    def unpack(parameters):
        point = np.array([parameters[0], parameters[1], np.exp(parameters[2])])
        normal = np.array([parameters[3], parameters[4], 1.])
        normal /= np.linalg.norm(normal)
        return point, normal

    def residual(parameters):
        point, normal = unpack(parameters)
        distance = normal@point
        model = (distance**2*np.eye(3)-distance*(np.outer(normal, point)+np.outer(point, normal))
                 +(point@point-radius_m**2)*np.outer(normal, normal))
        model /= np.linalg.norm(model)
        return ((model-observed)[indices])*weights

    candidates = []
    for normal in initials:
        if normal[2] < 0:
            normal = -normal
        if normal[2] < .02:
            continue
        start = [*(np.r_[center, 1.]*initial_z)[:2], np.log(initial_z), *(normal[:2]/normal[2])]
        solved = least_squares(residual, start,
            bounds=([-2, -2, np.log(.1), -50, -50], [2, 2, np.log(2.), 50, 50]),
            max_nfev=150, ftol=1e-11, xtol=1e-11, gtol=1e-11)
        point, normal = unpack(solved.x)
        if not solved.success or not np.isfinite(point).all():
            continue
        basis = np.cross(normal, [1., 0, 0])
        if np.linalg.norm(basis) < .1:
            basis = np.cross(normal, [0., 1, 0])
        basis /= np.linalg.norm(basis)
        other = np.cross(normal, basis)
        angles = np.linspace(0, 2*np.pi, 96, endpoint=False)
        rim = point+radius_m*(np.cos(angles)[:, None]*basis+np.sin(angles)[:, None]*other)
        if np.any(rim[:, 2] <= 0):
            continue
        projected = rim@k.T
        projected = projected[:, :2]/projected[:, 2:3]
        q = np.c_[projected, np.ones(len(projected))]
        gradient = 2*(q@observed_pixel.T)[:, :2]
        distances = np.abs(np.einsum('ij,ij->i', q@observed_pixel, q))/np.maximum(np.linalg.norm(gradient, axis=1), 1e-12)
        error = float(np.percentile(distances, 95))
        if error > 2:
            continue
        if any(np.linalg.norm(point-old['point']) < .0001 and normal@old['normal'] > .9999 for old in candidates):
            continue
        candidates.append(dict(point=point, normal=normal, rim=rim,
                               circle_reprojection_p95_px=error, solver_nfev=int(solved.nfev)))
    if not candidates:
        raise ValueError('Known-radius circle pose did not converge; no motion candidate')
    return candidates


def locate(frame, bbox, intrinsics, diameter_mm, total_height_mm):
    k = np.asarray(intrinsics, float)
    radius, height = float(diameter_mm)/2000, float(total_height_mm)/1000
    if not 0 < radius < height or k.shape != (3, 3):
        raise ValueError('Measured cylinder dimensions and calibrated K required')
    ellipse, detail = cap_ellipse(frame, bbox)
    roots = circle_pose_candidates(ellipse, k, radius)
    x, y, w, h = [float(v) for v in bbox]
    observed_bbox = np.array([x, y, x+w, y+h])
    choices = []
    for root in roots:
        def evaluate(visible_height):
            # Visible height is fitted only to the image silhouette: recess
            # insertion is unknown and total98mm is not assumed fully exposed.
            cloud = np.vstack([root['rim'], root['rim']+visible_height*root['normal']])
            if np.any(cloud[:, 2] <= 0):
                return 1e6
            pixels = cloud@k.T
            pixels = pixels[:, :2]/pixels[:, 2:3]
            box = np.r_[pixels.min(axis=0), pixels.max(axis=0)]
            return float(np.sqrt(np.mean((box-observed_bbox)**2)))
        fitted = minimize_scalar(evaluate, bounds=(.005, height), method='bounded', options={'maxiter': 40})
        choices.append(dict(top_center_camera_m=root['point'].tolist(),
            axis_from_top_into_body_camera=root['normal'].tolist(),
            geometric_body_center_camera_m=(root['point']+.5*height*root['normal']).tolist(),
            visible_height_estimated_from_silhouette_mm=float(fitted.x*1000),
            body_bbox_rms_px=float(fitted.fun),circle_reprojection_p95_px=root['circle_reprojection_p95_px']))
    choices.sort(key=lambda row: row['body_bbox_rms_px'])
    best = choices[0]
    maximum_bbox_rms = max(3., .03*max(w, h))
    quality = best['body_bbox_rms_px'] <= maximum_bbox_rms
    ambiguous = False
    if len(choices) > 1:
        a = np.array(best['axis_from_top_into_body_camera'])
        b = np.array(choices[1]['axis_from_top_into_body_camera'])
        ambiguous = a@b < .999 and choices[1]['body_bbox_rms_px']-best['body_bbox_rms_px'] < 2
    result = dict(method='measured_circle_conic_and_visible_cylinder_silhouette',
        ellipse=dict(center_px=list(ellipse[0]),diameters_px=list(ellipse[1]),angle_deg=ellipse[2]),
        contour_detail=detail,pose_branches=choices,geometry_candidate_available=quality and not ambiguous,
        reject_reason='ambiguous_circle_pose' if ambiguous else (None if quality else 'body_silhouette_mismatch'),
        diameter_mm=diameter_mm,total_height_mm=total_height_mm,
        thresholds_scope='new_image_quality_defaults_not_metrological_acceptance',
        maximum_body_bbox_rms_px=maximum_bbox_rms,independent_object_position_accepted=False,
        per_slot_lookup=False,RGBD_depth_used=False,execution_ready=False)
    if result['geometry_candidate_available']:
        result['selected_candidate'] = best
    return result
