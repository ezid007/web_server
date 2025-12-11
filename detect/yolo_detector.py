"""
YOLO 감지 모듈
터틀봇 카메라 및 CCTV에서 사람 감지
웹에서 ON/OFF 제어 가능
"""

import cv2
import threading
from pathlib import Path
from datetime import datetime
from ultralytics import YOLO


class YoloDetector:
    """
    YOLO 기반 사람 감지기
    - 터틀봇 카메라, CCTV 프레임에서 사람 감지
    - 웹에서 실시간 ON/OFF 제어
    """
    
    def __init__(self, model_path: str = None):
        """
        Args:
            model_path: YOLO 모델 경로 (None이면 기본 모델 사용)
        """
        self._enabled = False
        self._lock = threading.Lock()
        self._model = None
        self._model_path = model_path
        
        # 기본 모델 경로 설정
        if model_path is None:
            base_dir = Path(__file__).parent.parent
            # 학습된 모델 우선, 없으면 기본 모델
            custom_model = base_dir / "models" / "my_yolo.pt"
            if custom_model.exists():
                self._model_path = str(custom_model)
            else:
                self._model_path = str(base_dir / "yolo11n.pt")
        
        self._confidence = 0.5
    
    @property
    def enabled(self) -> bool:
        """YOLO 활성화 상태"""
        with self._lock:
            return self._enabled
    
    @enabled.setter
    def enabled(self, value: bool):
        with self._lock:
            self._enabled = value
            if value and self._model is None:
                self._load_model()
    
    def _load_model(self):
        """YOLO 모델 로드"""
        try:
            print(f"🔄 YOLO 모델 로딩: {self._model_path}")
            self._model = YOLO(self._model_path)
            print("✅ YOLO 모델 로드 완료")
        except Exception as e:
            print(f"❌ YOLO 모델 로드 실패: {e}")
            self._model = None
    
    def toggle(self) -> bool:
        """YOLO ON/OFF 토글, 새 상태 반환"""
        with self._lock:
            self._enabled = not self._enabled
            if self._enabled and self._model is None:
                self._load_model()
            return self._enabled
    
    def detect_persons(self, frame) -> list:
        """
        프레임에서 사람 감지
        
        Args:
            frame: OpenCV BGR 이미지
            
        Returns:
            감지된 사람 목록 [{"bbox": (x1,y1,x2,y2), "confidence": float}, ...]
        """
        if not self._enabled or self._model is None or frame is None:
            return []
        
        try:
            # YOLO 추론 (사람 클래스만)
            results = self._model(frame, conf=self._confidence, classes=[0], verbose=False)
            
            persons = []
            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    conf = float(box.conf[0].cpu().numpy())
                    persons.append({
                        "bbox": (int(x1), int(y1), int(x2), int(y2)),
                        "confidence": conf
                    })
            
            return persons
            
        except Exception as e:
            print(f"⚠️ YOLO 감지 오류: {e}")
            return []
    
    def draw_detections(self, frame, detections: list):
        """
        프레임에 감지 결과 그리기
        
        Args:
            frame: OpenCV BGR 이미지
            detections: detect_persons()의 반환값
            
        Returns:
            바운딩 박스가 그려진 프레임
        """
        if frame is None:
            return frame
            
        display = frame.copy()
        
        for det in detections:
            x1, y1, x2, y2 = det["bbox"]
            conf = det["confidence"]
            
            # 바운딩 박스
            color = (0, 255, 0) if conf > 0.7 else (0, 255, 255)
            cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
            
            # 레이블
            label = f"Person: {conf:.2f}"
            cv2.putText(display, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        return display


# 전역 YOLO 감지기 인스턴스
yolo_detector = YoloDetector()
