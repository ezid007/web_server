#!/usr/bin/env python3
"""
사용자 추적 스크립트 (ROS2 독립 버전)
학습된 YOLO 모델을 사용하여 사용자를 추적하고
TurtleBot3를 제어합니다.
"""

import os
import cv2
import time
import json
import argparse
import requests
from pathlib import Path
from ultralytics import YOLO
from dotenv import load_dotenv

# 프로젝트 경로
PROJECT_DIR = Path(__file__).parent.parent
WEB_SERVER_DIR = PROJECT_DIR.parent
MODELS_DIR = PROJECT_DIR / "models" / "user_detection" / "weights"

# .env 파일 로드
load_dotenv(WEB_SERVER_DIR / ".env")

# 환경 변수에서 TurtleBot IP 가져오기 (로봇 제어용)
TURTLEBOT_IP = os.getenv("TURTLEBOT_IP")
# 카메라 스트림과 API는 로컬 web_server에서 제공
WEB_SERVER_PORT = os.getenv("PORT")
DEFAULT_STREAM_URL = f"http://localhost:{WEB_SERVER_PORT}/camera/stream"
DEFAULT_API_URL = f"http://localhost:{WEB_SERVER_PORT}"


class UserFollower:
    """사용자 추적 클래스"""
    
    def __init__(
        self,
        model_path: str,
        stream_url: str,
        robot_api_url: str,
        target_class: int = 1,
        confidence_threshold: float = 0.5
    ):
        """
        Args:
            model_path: YOLO 모델 경로
            stream_url: 카메라 스트림 URL
            robot_api_url: 로봇 제어 API URL (대시보드)
            target_class: 추적할 클래스 ID
            confidence_threshold: 탐지 신뢰도 임계값
        """
        self.stream_url = stream_url
        self.robot_api_url = robot_api_url
        self.target_class = target_class
        self.confidence_threshold = confidence_threshold
        
        # YOLO 모델 로드
        print(f"🔄 모델 로딩: {model_path}")
        self.model = YOLO(model_path)
        
        # 제어 파라미터
        self.image_width = 640
        self.image_height = 480
        self.target_bbox_width = 180  # 목표 바운딩 박스 너비 (픽셀)
        
        # PID 제어 파라미터
        self.kp_angular = 0.004  # 회전 P 게인
        self.kp_linear = 0.003   # 직진 P 게인
        self.max_linear = 0.15   # 최대 직진 속도 (m/s)
        self.max_angular = 0.4   # 최대 회전 속도 (rad/s)
        
        # 상태
        self.following = False
        self.last_detection_time = 0
        self.search_mode = False
    
    def send_velocity(self, linear: float, angular: float):
        """로봇에 속도 명령 전송 (대시보드 API 활용)"""
        try:
            # 속도 제한
            linear = max(-self.max_linear, min(self.max_linear, linear))
            angular = max(-self.max_angular, min(self.max_angular, angular))
            
            # 대시보드의 teleop API 호출
            response = requests.post(
                f"{self.robot_api_url}/teleop",
                json={"linear_x": linear, "angular_z": angular},
                timeout=0.5
            )
            return response.status_code == 200
        except Exception as e:
            print(f"⚠️ 속도 명령 전송 실패: {e}")
            return False
    
    def stop_robot(self):
        """로봇 정지"""
        self.send_velocity(0.0, 0.0)
    
    def calculate_control(self, bbox):
        """바운딩 박스로부터 제어 명령 계산"""
        x1, y1, x2, y2 = bbox
        
        # 중심점 계산
        center_x = (x1 + x2) / 2
        bbox_width = x2 - x1
        
        # 좌우 오차 (화면 중앙과의 차이)
        error_x = center_x - (self.image_width / 2)
        angular = -error_x * self.kp_angular
        
        # 거리 오차 (바운딩 박스 크기 기반)
        error_distance = self.target_bbox_width - bbox_width
        linear = error_distance * self.kp_linear
        
        # 너무 가까우면 후진
        if bbox_width > self.target_bbox_width * 1.5:
            linear = -0.1
        
        return linear, angular
    
    def run(self):
        """추적 실행"""
        print(f"\n📷 카메라 스트림 연결: {self.stream_url}")
        cap = cv2.VideoCapture(self.stream_url)
        
        if not cap.isOpened():
            print("❌ 카메라 스트림을 열 수 없습니다!")
            return
        
        print("\n" + "=" * 50)
        print("🎯 사용자 추적 모드 시작!")
        print("=" * 50)
        print("조작:")
        print("  [SPACE] - 추적 시작/정지")
        print("  [S] - 긴급 정지")
        print("  [Q] - 종료")
        print("=" * 50 + "\n")
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print("⚠️ 프레임 읽기 실패")
                time.sleep(0.1)
                continue
            
            self.image_height, self.image_width = frame.shape[:2]
            
            # YOLO 추론
            results = self.model(
                frame,
                conf=self.confidence_threshold,
                classes=[self.target_class],
                verbose=False
            )
            
            display = frame.copy()
            target_detected = False
            
            # 탐지 결과 처리
            for result in results:
                boxes = result.boxes
                if len(boxes) > 0:
                    # 가장 큰 (가장 가까운) 박스 선택
                    best_box = None
                    best_area = 0
                    
                    for box in boxes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                        area = (x2 - x1) * (y2 - y1)
                        
                        if area > best_area:
                            best_area = area
                            best_box = (x1, y1, x2, y2)
                    
                    if best_box:
                        x1, y1, x2, y2 = best_box
                        target_detected = True
                        self.last_detection_time = time.time()
                        self.search_mode = False
                        
                        # 추적 중일 때만 제어
                        if self.following:
                            linear, angular = self.calculate_control(best_box)
                            self.send_velocity(linear, angular)
                        
                        # 바운딩 박스 그리기
                        color = (0, 255, 0) if self.following else (0, 255, 255)
                        cv2.rectangle(display, (x1, y1), (x2, y2), color, 3)
                        
                        # 정보 표시
                        names = self.model.names
                        class_name = names.get(self.target_class, f"ID:{self.target_class}")
                        conf = boxes[0].conf[0].cpu().numpy()
                        cv2.putText(display, f"{class_name}: {conf:.2f}", (x1, y1 - 10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            
            # 타겟 없음 - 정지 또는 탐색
            if not target_detected:
                if self.following:
                    time_since_detection = time.time() - self.last_detection_time
                    
                    if time_since_detection > 2.0:
                        # 2초 이상 탐지 못함 - 탐색 모드
                        if not self.search_mode:
                            print("🔍 탐색 모드 진입...")
                            self.search_mode = True
                        
                        # 제자리 회전으로 탐색
                        self.send_velocity(0.0, 0.3)
                    else:
                        # 잠시 대기
                        self.stop_robot()
                else:
                    self.stop_robot()
            
            # 상태 표시
            status = "FOLLOWING" if self.following else "STANDBY"
            color = (0, 255, 0) if self.following else (128, 128, 128)
            cv2.putText(display, f"Mode: {status}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            
            if self.search_mode:
                cv2.putText(display, "SEARCHING...", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            cv2.imshow("User Following (Q to quit)", display)
            
            # 키 입력
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                self.stop_robot()
                break
            elif key == ord(' '):
                self.following = not self.following
                if not self.following:
                    self.stop_robot()
                print(f"{'🏃 추적 시작!' if self.following else '⏹️ 추적 중지'}")
            elif key == ord('s'):
                self.following = False
                self.stop_robot()
                print("🛑 긴급 정지!")
        
        cap.release()
        cv2.destroyAllWindows()
        print("\n✅ 추적 종료")


def main():
    parser = argparse.ArgumentParser(description="사용자 추적 시스템")
    parser.add_argument(
        "--model", "-m",
        default=str(MODELS_DIR / "best.pt"),
        help="YOLO 모델 경로"
    )
    parser.add_argument(
        "--stream", "-s",
        default=DEFAULT_STREAM_URL,
        help="카메라 스트림 URL"
    )
    parser.add_argument(
        "--robot-api", "-r",
        default=DEFAULT_API_URL,
        help="로봇 제어 API URL"
    )
    parser.add_argument(
        "--class-id", "-c",
        type=int,
        default=1,
        help="추적할 클래스 ID"
    )
    parser.add_argument(
        "--confidence", "-t",
        type=float,
        default=0.5,
        help="탐지 신뢰도 임계값"
    )
    
    args = parser.parse_args()
    
    # 모델 파일 확인
    if not Path(args.model).exists():
        print(f"❌ 모델 파일을 찾을 수 없습니다: {args.model}")
        print("   먼저 train.py로 모델을 학습시켜주세요.")
        return
    
    follower = UserFollower(
        model_path=args.model,
        stream_url=args.stream,
        robot_api_url=args.robot_api,
        target_class=args.class_id,
        confidence_threshold=args.confidence
    )
    
    follower.run()


if __name__ == "__main__":
    main()
