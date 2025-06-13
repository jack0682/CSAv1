# csa_yolo_inference/input_handler.py

import cv2
from pathlib import Path

class ImageInputHandler:
    """
    [🔧 CSA 테스트 전용 입력 핸들러]
    목적: ROS 없이도 YOLO 등의 인식기를 단독 테스트할 수 있는 유틸리티.
    운영 시스템에서는 image_publisher_node를 통해 topic 기반 입력을 사용하므로,
    이 모듈은 디버깅 또는 개발용 스크립트에서만 사용된다.
    """

    def __init__(self, mode: str = "dataset", source_path: str = None, loop: bool = False):
        self.mode = mode
        self.loop = loop
        self.current_index = 0

        if self.mode == "dataset":
            if source_path is None:
                raise ValueError("[❌] Dataset path must be provided.")
            image_dir = Path(source_path)
            if not image_dir.exists():
                raise FileNotFoundError(f"[❌] Not found: {image_dir}")
            self.image_paths = sorted(image_dir.glob("*.jpg")) + sorted(image_dir.glob("*.png"))
            if not self.image_paths:
                raise RuntimeError("[❌] No images found.")
        elif self.mode == "camera":
            cam_index = int(source_path) if source_path else 0
            self.cap = cv2.VideoCapture(cam_index)
            if not self.cap.isOpened():
                raise RuntimeError(f"[❌] Failed to open camera {cam_index}")
        else:
            raise ValueError(f"[❌] Unsupported mode: {self.mode}")

    def get_next_frame(self):
        if self.mode == "dataset":
            if self.current_index >= len(self.image_paths):
                if self.loop:
                    self.current_index = 0
                else:
                    return None
            frame = cv2.imread(str(self.image_paths[self.current_index]))
            self.current_index += 1
            return frame
        elif self.mode == "camera":
            ret, frame = self.cap.read()
            return frame if ret else None

    def release(self):
        if self.mode == "camera" and hasattr(self, 'cap'):
            self.cap.release()
