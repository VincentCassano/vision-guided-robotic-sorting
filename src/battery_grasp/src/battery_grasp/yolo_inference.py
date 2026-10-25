"""Local YOLOv5 checkpoint inference, independent of ROS."""
import os
import sys
from pathlib import Path

NAMES = ['black_cyl_battery', 'red_cyl_battery', 'white_cyl_battery', 'square_battery']
CATEGORIES = ['NORMAL_BLACK', 'DEFECT_RED', 'DEFECT_WHITE', 'UNSUPPORTED_SQUARE']


class BatteryDetector:
    def __init__(self, repository, weights, device='cpu', confidence=0.25, iou=0.45, square_confidence=None, class_confidences=None):
        repository, weights = Path(repository).resolve(), Path(weights).resolve()
        if not (repository / 'models/common.py').is_file() or not weights.is_file():
            raise ValueError('Existing local YOLOv5 repository and checkpoint required')
        if not 0 < confidence < 1 or not 0 < iou < 1:
            raise ValueError('Thresholds must be between zero and one')
        self.square_confidence = confidence if square_confidence is None else float(square_confidence)
        if not confidence <= self.square_confidence < 1:
            raise ValueError('Square confidence must be at least the base confidence and below one')
        self.class_confidences = [confidence, confidence, confidence, self.square_confidence]
        if class_confidences is not None:
            if len(class_confidences) != 4 or any(not confidence <= float(v) < 1 for v in class_confidences):
                raise ValueError('Four class confidences at least the base confidence and below one required')
            self.class_confidences = list(map(float, class_confidences))
        os.environ['YOLOv5_AUTOINSTALL'] = 'false'
        os.environ['YOLO_AUTOINSTALL'] = 'false'
        os.environ['YOLO_OFFLINE'] = 'true'
        sys.path.insert(0, str(repository))
        import torch
        from models.common import DetectMultiBackend
        from utils.augmentations import letterbox
        from utils.general import non_max_suppression, scale_boxes
        from utils.torch_utils import select_device
        self.torch, self.letterbox = torch, letterbox
        self.nms, self.scale_boxes = non_max_suppression, scale_boxes
        self.device = select_device(device)
        self.model = DetectMultiBackend(str(weights), device=self.device, fp16=False)
        names = self.model.names
        actual = [names[i] for i in range(len(names))]
        if actual != NAMES:
            raise ValueError('Model class order does not match battery_v1: ' + repr(actual))
        self.confidence, self.iou = confidence, iou

    def detect(self, bgr):
        import numpy as np
        if bgr is None or bgr.dtype != np.uint8 or bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError('Expected a uint8 BGR image')
        resized = self.letterbox(bgr, 640, stride=int(self.model.stride), auto=True)[0]
        tensor = self.torch.from_numpy(np.ascontiguousarray(resized.transpose(2, 0, 1)[::-1]))
        tensor = tensor.to(self.device).float().unsqueeze(0) / 255.0
        with self.torch.inference_mode():
            result = self.nms(self.model(tensor), self.confidence, self.iou, max_det=1000)[0]
            result[:, :4] = self.scale_boxes(tensor.shape[2:], result[:, :4], bgr.shape).round()
        detections = []
        for x1, y1, x2, y2, confidence, class_id in result.cpu().tolist():
            class_id = int(class_id)
            if confidence < self.class_confidences[class_id]:
                continue
            detections.append({'class_id': class_id, 'class_name': NAMES[class_id],
                               'category': CATEGORIES[class_id], 'grasp_supported': class_id != 3,
                               'confidence': confidence, 'u': (x1+x2)/2, 'v': (y1+y2)/2,
                               'bbox': [x1, y1, x2-x1, y2-y1]})
        return detections


def detection_message(detections, stamp_ns, frame_id, image_width=None, image_height=None):
    message = {'schema_version': 1, 'stamp': stamp_ns / 1e9, 'stamp_ns': stamp_ns,
               'frame_id': frame_id, 'detections': detections}
    if image_width is not None and image_height is not None:
        message.update(image_width=image_width, image_height=image_height)
    return message
