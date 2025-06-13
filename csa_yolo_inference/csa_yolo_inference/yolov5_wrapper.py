# csa_yolo_inference/yolov5_wrapper.py

import torch
import numpy as np
import cv2
from pathlib import Path

class DetectionResult:
    def __init__(self, class_id, label, confidence, bbox, feature=None, track_id=None):
        self.class_id = class_id
        self.label = label
        self.confidence = confidence
        self.xmin, self.ymin, self.xmax, self.ymax = bbox

        # Optional: 3D position placeholder (for SLAM fusion later)
        self.x3d = 0.0
        self.y3d = 0.0
        self.z3d = 0.0

        # Optional: velocity placeholder
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        # Optional: 128D embedding vector
        self.feature = feature if feature is not None else np.zeros(128, dtype=np.float32)

        # For ontology extension
        self.relations = []
        
        # ✅ 추가: 추적 ID
        self.track_id = track_id


class Yolov5Detector:
    def __init__(self, model_config: dict):
        weights_path = model_config['yolo_weights']
        conf_thres = model_config.get('conf_threshold', 0.4)
        iou_thres = model_config.get('iou_threshold', 0.5)

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=weights_path)
        self.model.conf = conf_thres
        self.model.iou = iou_thres
        self.model.to(self.device)
        self.model.eval()

        print(f"[🎯] YOLOv5 model loaded on {self.device.upper()}")
        print(f"     Conf Threshold: {self.model.conf}, IOU Threshold: {self.model.iou}")

    def infer(self, frame) -> list:
        """
        Run inference on a single RGB frame.
        Returns list of DetectionResult
        """
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.model(img_rgb, size=640)

        detections = []
        if results is None or results.xyxy[0] is None:
            return detections

        for *xyxy, conf, cls_id in results.xyxy[0].cpu().numpy():
            xmin, ymin, xmax, ymax = map(int, xyxy)
            confidence = float(conf)
            class_id = int(cls_id)
            label = self.model.names[class_id]

            bbox = (xmin, ymin, xmax, ymax)
            det = DetectionResult(class_id, label, confidence, bbox)
            detections.append(det)

        return detections
