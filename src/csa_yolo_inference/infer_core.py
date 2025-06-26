# infer_core.py

import torch
import numpy as np
import cv2
from pathlib import Path
from yolov5.models.yolo import Model
from yolov5.utils.datasets import letterbox
from yolov5.utils.general import non_max_suppression, scale_coords


class CSA_YOLOv5:
    def __init__(self, model_cfg='yolov5s_csa.yaml', device='cuda', conf_thres=0.3, iou_thres=0.45):
        self.device = device
        self.model = Model(model_cfg).to(device).eval()
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres

    def preprocess(self, img, img_size=640):
        img0 = img.copy()
        img = letterbox(img, img_size, stride=32, auto=True)[0]
        img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        img = np.ascontiguousarray(img)
        img_tensor = torch.from_numpy(img).to(self.device).float() / 255.0
        return img_tensor.unsqueeze(0), img0

    def infer(self, img):
        img_tensor, img0 = self.preprocess(img)
        with torch.no_grad():
            dets, features = self.model(img_tensor)  # 🔥 DetectWithFeature

        dets = non_max_suppression(dets, self.conf_thres, self.iou_thres)[0]  # (N, 6)
        results = []

        if dets is not None and len(dets):
            dets[:, :4] = scale_coords(img_tensor.shape[2:], dets[:, :4], img0.shape).round()

            for *xyxy, conf, cls in dets:
                x1, y1, x2, y2 = map(int, xyxy)
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

                feature_vector = self.extract_feature_vector(features, cx, cy, img0.shape)
                results.append({
                    'bbox': [x1, y1, x2, y2],
                    'confidence': float(conf),
                    'class_id': int(cls),
                    'feature_vector': feature_vector
                })

        return results

    def extract_feature_vector(self, feature_maps, cx, cy, image_shape):
        fmap = feature_maps[0]  # P3 기준
        ny, nx = fmap.shape[2:]
        fx = int(cx / image_shape[1] * nx)
        fy = int(cy / image_shape[0] * ny)
        fx = np.clip(fx, 0, nx - 1)
        fy = np.clip(fy, 0, ny - 1)
        v = fmap[0, :, fy, fx]  # (128,)
        return v.cpu().numpy().tolist()
