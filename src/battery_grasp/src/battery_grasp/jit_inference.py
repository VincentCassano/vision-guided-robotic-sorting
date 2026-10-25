"""Standalone TorchScript inference using torch, numpy and OpenCV only."""
import cv2
import numpy as np
import torch
from .yolo_inference import NAMES, CATEGORIES


class BatteryDetector:
    def __init__(self, repository, weights, device='cpu', confidence=.25, iou=.45, square_confidence=None, class_confidences=None):
        if device == 'auto': device='cuda:0' if torch.cuda.is_available() else 'cpu'
        if str(device).isdigit(): device='cuda:'+str(device)
        self.device=torch.device(device)
        self.model=torch.jit.load(str(weights),map_location=self.device).eval()
        self.confidence,self.iou=confidence,iou
        self.class_confidences=class_confidences or [confidence,confidence,confidence,square_confidence if square_confidence is not None else confidence]
        if len(self.class_confidences)!=4 or any(not confidence<=v<1 for v in self.class_confidences):
            raise ValueError('Invalid class thresholds')

    def detect(self,bgr):
        if bgr is None or bgr.dtype!=np.uint8 or bgr.ndim!=3 or bgr.shape[2]!=3: raise ValueError('Expected uint8 BGR image')
        h,w=bgr.shape[:2];r=min(640/h,640/w)
        nw,nh=round(w*r),round(h*r)
        dw,dh=(640-nw)%32/2,(640-nh)%32/2
        im=cv2.resize(bgr,(nw,nh)) if (nw,nh)!=(w,h) else bgr
        im=cv2.copyMakeBorder(im,round(dh-.1),round(dh+.1),round(dw-.1),round(dw+.1),cv2.BORDER_CONSTANT,value=(114,114,114))
        tensor=torch.from_numpy(np.ascontiguousarray(im.transpose(2,0,1)[::-1])).to(self.device).float().unsqueeze(0)/255
        with torch.inference_mode(): prediction=self.model(tensor)[0].cpu().numpy()
        prediction=prediction[prediction[:,4]>self.confidence]
        if not len(prediction): return []
        scores=prediction[:,5:]*prediction[:,4:5]
        classes=scores.argmax(1);conf=scores[np.arange(len(scores)),classes]
        valid=conf>self.confidence;prediction,classes,conf=prediction[valid],classes[valid],conf[valid]
        boxes=np.concatenate((prediction[:,:2]-prediction[:,2:4]/2,prediction[:,:2]+prediction[:,2:4]/2),axis=1)
        order=np.argsort(-conf,kind='stable')[:30000];keep=[]
        while len(order) and len(keep)<1000:
            i=order[0];keep.append(i);others=order[1:]
            lo=np.maximum(boxes[i,:2],boxes[others,:2]);hi=np.minimum(boxes[i,2:],boxes[others,2:])
            area=np.maximum(hi-lo,0).prod(1)
            overlap=area/((boxes[i,2:]-boxes[i,:2]).prod()+(boxes[others,2:]-boxes[others,:2]).prod(1)-area+1e-12)
            order=others[(classes[others]!=classes[i])|(overlap<=self.iou)]
        result=[]
        # Match YOLO scale_boxes: padding is computed from the actual tensor dimensions.
        gain=min(im.shape[0]/h,im.shape[1]/w)
        pad_x,pad_y=(im.shape[1]-w*gain)/2,(im.shape[0]-h*gain)/2
        for i in keep:
            c=int(classes[i]);confidence=float(conf[i])
            if confidence<self.class_confidences[c]: continue
            x1,y1,x2,y2=boxes[i]
            x1,x2=np.rint(np.clip((np.array([x1,x2])-pad_x)/gain,0,w)).tolist()
            y1,y2=np.rint(np.clip((np.array([y1,y2])-pad_y)/gain,0,h)).tolist()
            result.append(dict(class_id=c,class_name=NAMES[c],category=CATEGORIES[c],grasp_supported=c!=3,confidence=confidence,
                               u=(x1+x2)/2,v=(y1+y2)/2,bbox=[x1,y1,x2-x1,y2-y1]))
        return result
