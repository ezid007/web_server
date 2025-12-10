#!/usr/bin/env python3
"""
캡처 및 자동 라벨링 스크립트
TurtleBot3 카메라에서 이미지를 캡처하고
YOLOv11n으로 사람을 탐지하여 자동으로 라벨을 생성합니다.

두 가지 모드 지원:
1. ROS2 직접 구독 모드 (--ros2): ROS2 토픽에서 직접 영상 수신 (권장, 빠름)
2. HTTP 스트림 모드 (기본): web_server의 /camera/raw 엔드포인트 사용
"""

import os
import cv2
import yaml
import time
import argparse
import threading
import queue
from datetime import datetime
from pathlib import Path
from ultralytics import YOLO
from dotenv import load_dotenv

# 프로젝트 경로 설정
PROJECT_DIR = Path(__file__).parent.parent
WEB_SERVER_DIR = PROJECT_DIR.parent  # web_server 디렉토리
DATASET_DIR = PROJECT_DIR / "dataset"
IMAGES_DIR = DATASET_DIR / "images"
LABELS_DIR = DATASET_DIR / "labels"
CLASSES_FILE = PROJECT_DIR / "classes.yaml"

# .env 파일 로드
load_dotenv(WEB_SERVER_DIR / ".env")

# 환경 변수에서 설정 가져오기
TURTLEBOT_IP = os.getenv("TURTLEBOT_IP")
WEB_SERVER_PORT = os.getenv("PORT")
CAMERA_TOPIC = os.getenv("CAMERA_TOPIC", "/camera_node/image_raw")
DEFAULT_STREAM_URL = f"http://localhost:{WEB_SERVER_PORT}/camera/raw"


class ROS2CameraReader:
    """
    ROS2 토픽에서 직접 카메라 영상을 읽는 리더
    HTTP 오버헤드 없이 최고 성능 제공
    """

    def __init__(self, topic: str = None):
        self.topic = topic or CAMERA_TOPIC
        self.frame = None
        self.running = False
        self.lock = threading.Lock()
        self.node = None
        self.thread = None

    def start(self) -> bool:
        """ROS2 노드 시작"""
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import Image
            from cv_bridge import CvBridge

            self.rclpy = rclpy
            self.bridge = CvBridge()

            # ROS2 초기화 (이미 초기화되어 있으면 건너뜀)
            try:
                rclpy.init()
            except RuntimeError:
                pass  # 이미 초기화됨

            # 간단한 노드 클래스 정의
            class CameraNode(Node):
                def __init__(node_self, topic, callback):
                    super().__init__("auto_labeling_camera_node")
                    node_self.subscription = node_self.create_subscription(
                        Image, topic, callback, 10
                    )

            self.running = True
            self.node = CameraNode(self.topic, self._camera_callback)

            # 별도 스레드에서 spin
            self.thread = threading.Thread(target=self._spin_thread, daemon=True)
            self.thread.start()

            print(f"✅ ROS2 카메라 토픽 구독 시작: {self.topic}")
            return True

        except ImportError as e:
            print(f"❌ ROS2를 사용할 수 없습니다: {e}")
            return False
        except Exception as e:
            print(f"❌ ROS2 초기화 실패: {e}")
            return False

    def _spin_thread(self):
        """ROS2 spin을 별도 스레드에서 실행"""
        self.rclpy.spin(self.node)

    def _camera_callback(self, msg):
        """카메라 메시지 콜백"""
        try:
            if msg.encoding == "rgb8":
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # 640x480으로 리사이즈
            if cv_image.shape[1] != 640 or cv_image.shape[0] != 480:
                cv_image = cv2.resize(cv_image, (640, 480))

            with self.lock:
                self.frame = cv_image
        except Exception as e:
            print(f"⚠️ 프레임 변환 오류: {e}")

    def read(self):
        """최신 프레임 반환"""
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        """ROS2 노드 정지"""
        self.running = False
        if self.node:
            self.node.destroy_node()
        try:
            self.rclpy.shutdown()
        except:
            pass


class VideoStreamReader:
    """
    비동기 비디오 스트림 리더
    OpenCV의 버퍼링 문제를 해결하기 위해 별도 스레드에서 프레임을 읽음
    항상 최신 프레임만 유지하여 지연 방지
    """

    def __init__(self, stream_url: str):
        self.stream_url = stream_url
        self.frame = None
        self.running = False
        self.lock = threading.Lock()
        self.cap = None
        self.thread = None

    def start(self) -> bool:
        """스트림 시작"""
        self.cap = cv2.VideoCapture(self.stream_url)

        # MJPEG 스트림 최적화 설정
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 버퍼 최소화

        if not self.cap.isOpened():
            return False

        self.running = True
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()
        return True

    def _reader_loop(self):
        """프레임 읽기 루프 (별도 스레드)"""
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.frame = frame
            else:
                # 연결 끊김 시 재연결 시도
                time.sleep(0.5)
                self.cap.release()
                self.cap = cv2.VideoCapture(self.stream_url)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def read(self):
        """최신 프레임 반환 (None이면 프레임 없음)"""
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        """스트림 정지"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.cap:
            self.cap.release()


def load_classes():
    """classes.yaml에서 클래스 목록 로드"""
    if CLASSES_FILE.exists():
        with open(CLASSES_FILE, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
            return data.get("names", ["unknown"])
    return ["unknown"]


def save_classes(classes: list):
    """classes.yaml에 클래스 목록 저장"""
    data = {
        "path": str(DATASET_DIR),
        "train": "images",
        "val": "images",
        "nc": len(classes),
        "names": classes,
    }
    with open(CLASSES_FILE, "w", encoding="utf-8") as f:
        yaml.dump(data, f, allow_unicode=True, default_flow_style=False)


def add_new_person(name: str):
    """새로운 사람 클래스 추가"""
    classes = load_classes()
    if name not in classes:
        classes.append(name)
        save_classes(classes)
        print(f"✅ 새로운 클래스 추가됨: '{name}' (ID: {len(classes) - 1})")
    return classes.index(name)


def calculate_iou(box1, box2):
    """
    두 바운딩 박스의 IoU(Intersection over Union) 계산
    box 형식: (x_center, y_center, width, height) - 정규화된 좌표
    """
    # 박스를 (x1, y1, x2, y2) 형식으로 변환
    x1_1 = box1[0] - box1[2] / 2
    y1_1 = box1[1] - box1[3] / 2
    x2_1 = box1[0] + box1[2] / 2
    y2_1 = box1[1] + box1[3] / 2

    x1_2 = box2[0] - box2[2] / 2
    y1_2 = box2[1] - box2[3] / 2
    x2_2 = box2[0] + box2[2] / 2
    y2_2 = box2[1] + box2[3] / 2

    # 교집합 영역 계산
    inter_x1 = max(x1_1, x1_2)
    inter_y1 = max(y1_1, y1_2)
    inter_x2 = min(x2_1, x2_2)
    inter_y2 = min(y2_1, y2_2)

    if inter_x2 <= inter_x1 or inter_y2 <= inter_y1:
        return 0.0

    inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1)

    # 합집합 영역 계산
    box1_area = box1[2] * box1[3]
    box2_area = box2[2] * box2[3]
    union_area = box1_area + box2_area - inter_area

    if union_area == 0:
        return 0.0

    return inter_area / union_area


def is_scene_changed(current_detections, previous_detections, iou_threshold=0.85):
    """
    현재 탐지 결과가 이전과 충분히 다른지 확인
    
    Args:
        current_detections: 현재 프레임의 탐지 결과
        previous_detections: 이전 저장된 프레임의 탐지 결과
        iou_threshold: 이 값보다 IoU가 높으면 동일한 위치로 판단
    
    Returns:
        True: 장면이 변경됨 (저장해야 함)
        False: 장면이 유사함 (저장 건너뜀)
    """
    # 이전 탐지가 없으면 변경된 것으로 처리
    if not previous_detections:
        return True
    
    # 탐지 수가 다르면 변경된 것
    if len(current_detections) != len(previous_detections):
        return True
    
    # 각 탐지에 대해 IoU 확인
    for curr_det in current_detections:
        curr_box = (curr_det['x_center'], curr_det['y_center'], 
                    curr_det['width'], curr_det['height'])
        
        # 현재 탐지와 가장 유사한 이전 탐지 찾기
        max_iou = 0
        for prev_det in previous_detections:
            prev_box = (prev_det['x_center'], prev_det['y_center'],
                        prev_det['width'], prev_det['height'])
            iou = calculate_iou(curr_box, prev_box)
            max_iou = max(max_iou, iou)
        
        # IoU가 임계값보다 낮으면 위치가 변경된 것
        if max_iou < iou_threshold:
            return True
    
    # 모든 탐지가 유사한 위치에 있으면 변경 없음
    return False


def capture_and_label(
    stream_url: str,
    model_path: str = "yolo11n.pt",
    target_class_id: int = 0,
    save_interval: float = 1.0,
    confidence_threshold: float = 0.5,
    skip_frames: int = 0,
    use_ros2: bool = False,
):
    """
    카메라 스트림에서 이미지를 캡처하고 자동으로 라벨링

    Args:
        stream_url: MJPEG 스트림 URL (HTTP 모드에서 사용)
        model_path: YOLO 모델 경로
        target_class_id: 저장할 클래스 ID (0=unknown, 1=첫번째 사람, ...)
        save_interval: 저장 간격 (초)
        confidence_threshold: 탐지 신뢰도 임계값
        skip_frames: YOLO 추론을 건너뛸 프레임 수 (0=매 프레임 추론)
        use_ros2: ROS2 토픽에서 직접 읽기 (True=ROS2, False=HTTP)
    """
    import torch

    # 디렉토리 생성
    IMAGES_DIR.mkdir(parents=True, exist_ok=True)
    LABELS_DIR.mkdir(parents=True, exist_ok=True)

    # CUDA 확인
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"🖥️  디바이스: {device.upper()}")
    if device == "cuda":
        print(f"   GPU: {torch.cuda.get_device_name(0)}")
        print(
            f"   VRAM: {torch.cuda.get_device_properties(0).total_memory / 1024**3:.1f} GB"
        )

    # YOLO 모델 로드 (GPU 명시)
    print(f"🔄 YOLO 모델 로딩 중: {model_path}")
    model = YOLO(model_path)
    model.to(device)

    # 워밍업 (첫 추론은 느릴 수 있음)
    print("🔥 GPU 워밍업 중...")
    dummy = (
        cv2.imread(str(PROJECT_DIR / "scripts" / "warmup.jpg"))
        if (PROJECT_DIR / "scripts" / "warmup.jpg").exists()
        else None
    )
    if dummy is None:
        dummy = (torch.rand(1, 3, 480, 640) * 255).byte().numpy()[0].transpose(1, 2, 0)
    model(dummy, verbose=False)
    print("✅ 워밍업 완료")

    # 카메라 리더 시작 (ROS2 또는 HTTP)
    if use_ros2:
        print(f"📷 ROS2 카메라 토픽 연결 중: {CAMERA_TOPIC}")
        stream_reader = ROS2CameraReader(CAMERA_TOPIC)
    else:
        print(f"📷 HTTP 스트림 연결 중: {stream_url}")
        stream_reader = VideoStreamReader(stream_url)

    if not stream_reader.start():
        print("❌ 카메라 스트림을 열 수 없습니다!")
        return

    # 첫 프레임 대기
    print("⏳ 첫 프레임 대기 중...")
    for _ in range(30):  # 최대 3초 대기
        if stream_reader.read() is not None:
            break
        time.sleep(0.1)

    print("\n" + "=" * 50)
    print("🎯 자동 라벨링 시작!")
    print("=" * 50)
    print("조작 방법:")
    print("  [SPACE] - 현재 프레임 저장 (사람 탐지 시)")
    print("  [B] - 배경 이미지 저장 (빈 라벨)")
    print("  [A] - 자동 저장 모드 토글")
    print("  [N] - 새로운 사람 클래스 추가")
    print("  [1-9] - 클래스 ID 변경")
    print("  [Q] - 종료")
    print("=" * 50 + "\n")

    classes = load_classes()
    print(f"📋 현재 클래스 목록: {classes}")
    print(f"🏷️  현재 저장 클래스: {classes[target_class_id]} (ID: {target_class_id})")

    auto_save = False
    last_save_time = 0
    image_count = len(list(IMAGES_DIR.glob("*.jpg")))
    frame_count = 0
    last_detections = []
    last_saved_detections = []  # 마지막으로 저장된 탐지 결과
    skip_count = 0  # 유사도로 인해 건너뛴 횟수
    target_fps = 30  # 목표 FPS
    frame_interval = 1.0 / target_fps  # 프레임 간격 (약 33ms)
    last_frame_time = time.time()

    try:
        while True:
            # FPS 제한 (30 FPS)
            current_time = time.time()
            elapsed = current_time - last_frame_time
            if elapsed < frame_interval:
                time.sleep(frame_interval - elapsed)
                current_time = time.time()
            last_frame_time = current_time

            frame = stream_reader.read()
            if frame is None:
                time.sleep(0.01)
                continue

            frame_count += 1



            # YOLO 추론 (skip_frames에 따라 건너뛰기)
            if skip_frames == 0 or frame_count % (skip_frames + 1) == 0:
                results = model(
                    frame, conf=confidence_threshold, classes=[0], verbose=False
                )

                # 탐지 결과 처리
                detections = []
                for result in results:
                    boxes = result.boxes
                    for box in boxes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                        conf = box.conf[0].cpu().numpy()

                        h, w = frame.shape[:2]
                        x_center = ((x1 + x2) / 2) / w
                        y_center = ((y1 + y2) / 2) / h
                        box_width = (x2 - x1) / w
                        box_height = (y2 - y1) / h

                        detections.append(
                            {
                                "class_id": target_class_id,
                                "x_center": x_center,
                                "y_center": y_center,
                                "width": box_width,
                                "height": box_height,
                                "confidence": conf,
                                "bbox": (x1, y1, x2, y2),
                            }
                        )
                last_detections = detections
            else:
                detections = last_detections

            # 탐지 결과 그리기
            display_frame = frame.copy()
            for det in detections:
                x1, y1, x2, y2 = det["bbox"]
                conf = det["confidence"]

                color = (0, 255, 0) if conf > 0.7 else (0, 255, 255)
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)

                label = f"{classes[target_class_id]}: {conf:.2f}"
                cv2.putText(
                    display_frame,
                    label,
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    2,
                )

            # 상태 표시
            status_text = f"Class: {classes[target_class_id]} | Auto: {'ON' if auto_save else 'OFF'} | Saved: {image_count}"
            cv2.putText(
                display_frame,
                status_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )

            # 화면 표시
            cv2.imshow("Auto Labeling (Press Q to quit)", display_frame)

            # 자동 저장 모드
            should_save = False

            if auto_save and len(detections) > 0:
                if current_time - last_save_time >= save_interval:
                    # 유사도 체크: 장면이 변경되었을 때만 저장
                    if is_scene_changed(detections, last_saved_detections):
                        should_save = True
                        last_save_time = current_time
                    else:
                        skip_count += 1

            # 키 입력 처리
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break
            elif key == ord(" "):  # SPACE - 수동 저장
                if len(detections) > 0:
                    should_save = True
                else:
                    print("⚠️ 사람이 탐지되지 않았습니다!")
            elif key == ord("a"):  # A - 자동 저장 토글
                auto_save = not auto_save
                print(f"🔄 자동 저장 모드: {'ON' if auto_save else 'OFF'}")
            elif key == ord("n"):  # N - 새로운 사람 추가
                cv2.destroyAllWindows()
                name = input("새로운 사람 이름 입력: ").strip()
                if name:
                    target_class_id = add_new_person(name)
                    classes = load_classes()
                cv2.namedWindow("Auto Labeling (Press Q to quit)")
            elif key == ord("b"):  # B - 배경 이미지 저장 (빈 라벨)
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                image_filename = f"background_{timestamp}.jpg"
                label_filename = f"background_{timestamp}.txt"

                # 이미지 저장
                cv2.imwrite(str(IMAGES_DIR / image_filename), frame)

                # 빈 라벨 파일 저장
                with open(LABELS_DIR / label_filename, "w") as f:
                    pass  # 빈 파일

                image_count += 1
                print(f"🖼️  배경 저장됨: {image_filename}")
            elif ord("1") <= key <= ord("9"):  # 1-9 - 클래스 변경
                new_id = key - ord("1")
                if new_id < len(classes):
                    target_class_id = new_id
                    print(
                        f"🏷️  클래스 변경: {classes[target_class_id]} (ID: {target_class_id})"
                    )

            # 이미지 및 라벨 저장
            if should_save:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                class_name = classes[target_class_id]
                image_filename = f"{class_name}_{timestamp}.jpg"
                label_filename = f"{class_name}_{timestamp}.txt"

                # 이미지 저장
                cv2.imwrite(str(IMAGES_DIR / image_filename), frame)

                # 라벨 저장 (YOLO 포맷)
                with open(LABELS_DIR / label_filename, "w") as f:
                    for det in detections:
                        line = f"{det['class_id']} {det['x_center']:.6f} {det['y_center']:.6f} {det['width']:.6f} {det['height']:.6f}\n"
                        f.write(line)

                image_count += 1
                last_saved_detections = detections.copy()  # 저장된 탐지 결과 기록
                print(f"💾 저장됨: {image_filename} (탐지: {len(detections)}개)")

    finally:
        stream_reader.stop()
        cv2.destroyAllWindows()

    print(f"\n✅ 완료! 총 {image_count}장의 이미지가 저장되었습니다.")
    print(f"   📁 이미지: {IMAGES_DIR}")
    print(f"   📁 라벨: {LABELS_DIR}")


def main():
    parser = argparse.ArgumentParser(description="YOLO 기반 자동 라벨링 도구")
    parser.add_argument(
        "--stream", "-s", default=DEFAULT_STREAM_URL, help="카메라 스트림 URL"
    )
    parser.add_argument("--model", "-m", default="yolo11n.pt", help="YOLO 모델 경로")
    parser.add_argument(
        "--class-id",
        "-c",
        type=int,
        default=0,
        help="저장할 클래스 ID (기본: 0=unknown)",
    )
    parser.add_argument(
        "--interval", "-i", type=float, default=1.0, help="자동 저장 간격 (초)"
    )
    parser.add_argument(
        "--confidence", "-t", type=float, default=0.5, help="탐지 신뢰도 임계값"
    )
    parser.add_argument(
        "--skip-frames",
        "-k",
        type=int,
        default=0,
        help="YOLO 추론을 건너뛸 프레임 수 (0=매 프레임, 1=2프레임마다, 2=3프레임마다...)",
    )
    parser.add_argument(
        "--ros2",
        "-r",
        action="store_true",
        help="ROS2 토픽에서 직접 영상 수신 (권장, 빠름). web_server 없이 사용 가능",
    )
    parser.add_argument("--add-person", "-a", type=str, help="새로운 사람 클래스 추가")

    args = parser.parse_args()

    # 새로운 사람 추가
    if args.add_person:
        add_new_person(args.add_person)
        return

    # 캡처 및 라벨링 시작
    capture_and_label(
        stream_url=args.stream,
        model_path=args.model,
        target_class_id=args.class_id,
        save_interval=args.interval,
        confidence_threshold=args.confidence,
        skip_frames=args.skip_frames,
        use_ros2=args.ros2,
    )


if __name__ == "__main__":
    main()
