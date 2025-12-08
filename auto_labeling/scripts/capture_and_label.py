#!/usr/bin/env python3
"""
캡처 및 자동 라벨링 스크립트
TurtleBot3 카메라(MJPEG 스트림)에서 이미지를 캡처하고
YOLOv11n으로 사람을 탐지하여 자동으로 라벨을 생성합니다.
"""

import os
import cv2
import yaml
import time
import argparse
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

# 환경 변수에서 TurtleBot IP 가져오기 (로봇 제어용)
TURTLEBOT_IP = os.getenv("TURTLEBOT_IP")
# 카메라 스트림은 로컬 web_server에서 제공
WEB_SERVER_PORT = os.getenv("PORT")
DEFAULT_STREAM_URL = f"http://localhost:{WEB_SERVER_PORT}/camera/raw"


def load_classes():
    """classes.yaml에서 클래스 목록 로드"""
    if CLASSES_FILE.exists():
        with open(CLASSES_FILE, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
            return data.get('names', ['unknown'])
    return ['unknown']


def save_classes(classes: list):
    """classes.yaml에 클래스 목록 저장"""
    data = {
        'path': str(DATASET_DIR),
        'train': 'images',
        'val': 'images',
        'nc': len(classes),
        'names': classes
    }
    with open(CLASSES_FILE, 'w', encoding='utf-8') as f:
        yaml.dump(data, f, allow_unicode=True, default_flow_style=False)


def add_new_person(name: str):
    """새로운 사람 클래스 추가"""
    classes = load_classes()
    if name not in classes:
        classes.append(name)
        save_classes(classes)
        print(f"✅ 새로운 클래스 추가됨: '{name}' (ID: {len(classes) - 1})")
    return classes.index(name)


def capture_and_label(
    stream_url: str,
    model_path: str = "yolo11n.pt",
    target_class_id: int = 0,
    save_interval: float = 1.0,
    confidence_threshold: float = 0.5
):
    """
    카메라 스트림에서 이미지를 캡처하고 자동으로 라벨링
    
    Args:
        stream_url: MJPEG 스트림 URL (예: http://192.168.0.25:8000/video_feed_turtlebot)
        model_path: YOLO 모델 경로
        target_class_id: 저장할 클래스 ID (0=unknown, 1=첫번째 사람, ...)
        save_interval: 저장 간격 (초)
        confidence_threshold: 탐지 신뢰도 임계값
    """
    # 디렉토리 생성
    IMAGES_DIR.mkdir(parents=True, exist_ok=True)
    LABELS_DIR.mkdir(parents=True, exist_ok=True)
    
    # YOLO 모델 로드
    print(f"🔄 YOLO 모델 로딩 중: {model_path}")
    model = YOLO(model_path)
    
    # 카메라 스트림 열기
    print(f"📷 카메라 스트림 연결 중: {stream_url}")
    cap = cv2.VideoCapture(stream_url)
    
    if not cap.isOpened():
        print("❌ 카메라 스트림을 열 수 없습니다!")
        return
    
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
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️ 프레임 읽기 실패, 재연결 시도...")
            cap.release()
            time.sleep(1)
            cap = cv2.VideoCapture(stream_url)
            continue
        
        # YOLO 추론 (person 클래스만 탐지)
        results = model(frame, conf=confidence_threshold, classes=[0], verbose=False)
        
        # 탐지 결과 그리기
        display_frame = frame.copy()
        detections = []
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                conf = box.conf[0].cpu().numpy()
                
                # 바운딩 박스 그리기
                color = (0, 255, 0) if conf > 0.7 else (0, 255, 255)
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)
                
                # 라벨 표시
                label = f"{classes[target_class_id]}: {conf:.2f}"
                cv2.putText(display_frame, label, (x1, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                
                # YOLO 포맷으로 변환 (정규화된 x_center, y_center, width, height)
                h, w = frame.shape[:2]
                x_center = ((x1 + x2) / 2) / w
                y_center = ((y1 + y2) / 2) / h
                box_width = (x2 - x1) / w
                box_height = (y2 - y1) / h
                
                detections.append({
                    'class_id': target_class_id,
                    'x_center': x_center,
                    'y_center': y_center,
                    'width': box_width,
                    'height': box_height,
                    'confidence': conf
                })
        
        # 상태 표시
        status_text = f"Class: {classes[target_class_id]} | Auto: {'ON' if auto_save else 'OFF'} | Saved: {image_count}"
        cv2.putText(display_frame, status_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # 화면 표시
        cv2.imshow("Auto Labeling (Press Q to quit)", display_frame)
        
        # 자동 저장 모드
        current_time = time.time()
        should_save = False
        
        if auto_save and len(detections) > 0:
            if current_time - last_save_time >= save_interval:
                should_save = True
                last_save_time = current_time
        
        # 키 입력 처리
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            break
        elif key == ord(' '):  # SPACE - 수동 저장
            if len(detections) > 0:
                should_save = True
            else:
                print("⚠️ 사람이 탐지되지 않았습니다!")
        elif key == ord('a'):  # A - 자동 저장 토글
            auto_save = not auto_save
            print(f"🔄 자동 저장 모드: {'ON' if auto_save else 'OFF'}")
        elif key == ord('n'):  # N - 새로운 사람 추가
            cv2.destroyAllWindows()
            name = input("새로운 사람 이름 입력: ").strip()
            if name:
                target_class_id = add_new_person(name)
                classes = load_classes()
            cv2.namedWindow("Auto Labeling (Press Q to quit)")
        elif key == ord('b'):  # B - 배경 이미지 저장 (빈 라벨)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            image_filename = f"background_{timestamp}.jpg"
            label_filename = f"background_{timestamp}.txt"
            
            # 이미지 저장
            cv2.imwrite(str(IMAGES_DIR / image_filename), frame)
            
            # 빈 라벨 파일 저장
            with open(LABELS_DIR / label_filename, 'w') as f:
                pass  # 빈 파일
            
            image_count += 1
            print(f"🖼️  배경 저장됨: {image_filename}")
        elif ord('1') <= key <= ord('9'):  # 1-9 - 클래스 변경
            new_id = key - ord('1')
            if new_id < len(classes):
                target_class_id = new_id
                print(f"🏷️  클래스 변경: {classes[target_class_id]} (ID: {target_class_id})")
        
        # 이미지 및 라벨 저장
        if should_save:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            class_name = classes[target_class_id]
            image_filename = f"{class_name}_{timestamp}.jpg"
            label_filename = f"{class_name}_{timestamp}.txt"
            
            # 이미지 저장
            cv2.imwrite(str(IMAGES_DIR / image_filename), frame)
            
            # 라벨 저장 (YOLO 포맷)
            with open(LABELS_DIR / label_filename, 'w') as f:
                for det in detections:
                    line = f"{det['class_id']} {det['x_center']:.6f} {det['y_center']:.6f} {det['width']:.6f} {det['height']:.6f}\n"
                    f.write(line)
            
            image_count += 1
            print(f"💾 저장됨: {image_filename} (탐지: {len(detections)}개)")
    
    cap.release()
    cv2.destroyAllWindows()
    print(f"\n✅ 완료! 총 {image_count}장의 이미지가 저장되었습니다.")
    print(f"   📁 이미지: {IMAGES_DIR}")
    print(f"   📁 라벨: {LABELS_DIR}")


def main():
    parser = argparse.ArgumentParser(description="YOLO 기반 자동 라벨링 도구")
    parser.add_argument(
        "--stream", "-s",
        default=DEFAULT_STREAM_URL,
        help="카메라 스트림 URL"
    )
    parser.add_argument(
        "--model", "-m",
        default="yolo11n.pt",
        help="YOLO 모델 경로"
    )
    parser.add_argument(
        "--class-id", "-c",
        type=int,
        default=0,
        help="저장할 클래스 ID (기본: 0=unknown)"
    )
    parser.add_argument(
        "--interval", "-i",
        type=float,
        default=1.0,
        help="자동 저장 간격 (초)"
    )
    parser.add_argument(
        "--confidence", "-t",
        type=float,
        default=0.5,
        help="탐지 신뢰도 임계값"
    )
    parser.add_argument(
        "--add-person", "-a",
        type=str,
        help="새로운 사람 클래스 추가"
    )
    
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
        confidence_threshold=args.confidence
    )


if __name__ == "__main__":
    main()
