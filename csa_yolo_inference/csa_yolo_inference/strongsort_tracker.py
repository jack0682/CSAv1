import numpy as np
import torch
from pathlib import Path
from boxmot.trackers.strongsort.strongsort import StrongSort
from boxmot.utils.ops import xyxy2tlwh
from .yolov5_wrapper import DetectionResult


class StrongSortTracker:
    def __init__(self, tracking_config: dict):
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.reid_weights = Path(tracking_config.get("reid_weights", ""))  # 경로 지정 필수
        self.half = tracking_config.get("half", False)

        self.tracker = StrongSort(
            reid_weights=self.reid_weights,
            device=self.device,
            half=self.half,
            per_class=tracking_config.get("per_class", False),
            min_conf=tracking_config.get("min_conf", 0.1),
            max_cos_dist=tracking_config.get("max_cos_dist", 0.2),
            max_iou_dist=tracking_config.get("max_iou_dist", 0.7),
            max_age=tracking_config.get("max_age", 30),
            n_init=tracking_config.get("n_init", 3),
            nn_budget=tracking_config.get("nn_budget", 100),
            mc_lambda=tracking_config.get("mc_lambda", 0.98),
            ema_alpha=tracking_config.get("ema_alpha", 0.9),
        )
        
    def update(self, detections: list, frame: np.ndarray) -> list:
        if not detections:
            return []

        bboxes = np.array([[det.xmin, det.ymin, det.xmax, det.ymax] for det in detections], dtype=np.float32)
        confs = np.array([det.confidence for det in detections], dtype=np.float32)
        classes = np.array([det.class_id for det in detections], dtype=np.int32)

        # Compose detections in [x1, y1, x2, y2, conf, cls]
        dets = np.hstack([bboxes, confs[:, None], classes[:, None]])

        # Run update
        outputs = self.tracker.update(dets=dets, img=frame)

        # Format tracked results
        results = []
        for out in outputs:
            xmin, ymin, xmax, ymax, track_id, conf, class_id, det_ind = out
            results.append(DetectionResult(
                class_id=int(class_id),
                label=str(class_id),  # optional mapping
                confidence=float(conf),
                bbox=(int(xmin), int(ymin), int(xmax), int(ymax)),
                track_id=int(track_id),
            ))

        return results
