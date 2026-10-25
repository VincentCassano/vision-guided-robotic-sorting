"""Image workspace and duplicate identity only; never a grasp coordinate table."""
import math


def polygon(record,context):
    if record.get('schema_version')!=1 or record.get('camera_context')!=context:
        raise ValueError('Workspace camera/model/setup differs; define the workspace again')
    points=record.get('polygon_px')
    if (not isinstance(points,list) or len(points)!=4 or any(
            not isinstance(p,list) or len(p)!=2 or any(type(v) not in (int,float)
                or not math.isfinite(v) for v in p) or not 0<=p[0]<1280 or not 0<=p[1]<720 for p in points)):
        raise ValueError('Four ordered image points required')
    signs=[]
    for i in range(4):
        a,b,c=points[i],points[(i+1)%4],points[(i+2)%4]
        signs.append((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))
    if not (all(v>0 for v in signs) or all(v<0 for v in signs)):
        raise ValueError('Workspace must be a noncrossing convex quadrilateral')
    return points


def contains(point,points):
    signs=[(b[0]-a[0])*(point[1]-a[1])-(b[1]-a[1])*(point[0]-a[0])
        for a,b in zip(points,points[1:]+points[:1])]
    return all(v>=0 for v in signs) or all(v<=0 for v in signs)


def foot(box):
    x,y,w,h=box
    return [x+w/2.,y+h]


def iou(a,b):
    ax,ay,aw,ah=a;bx,by,bw,bh=b
    area=max(0,min(ax+aw,bx+bw)-max(ax,bx))*max(0,min(ay+ah,by+bh)-max(ay,by))
    return area/max(aw*ah+bw*bh-area,1e-9)


def same_detection(a,b):
    # Box overlap alone can confuse adjacent upright cylinders in successive
    # rows. Also require the base to remain within one observed cap radius.
    return iou(a,b)>=.25 and math.dist(foot(a),foot(b))<=min(a[2],b[2])/2.


def front_first(detection):
    x,y=foot(detection['bbox'])
    return (-y,x)
