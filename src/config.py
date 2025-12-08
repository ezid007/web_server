"""
ROS2 토픽 및 설정값 관리
하드코딩 제거를 위해 모든 설정을 이 파일에서 관리
"""
import os
from pathlib import Path
from dotenv import load_dotenv

load_dotenv()

# 카메라 토픽
CAMERA_TOPIC = os.getenv("CAMERA_TOPIC")

# SLAM 지도 토픽
MAP_TOPIC = os.getenv("MAP_TOPIC")

# 텔레옵 명령 토픽
CMD_VEL_TOPIC = os.getenv("CMD_VEL_TOPIC")

# 시스템 정보 토픽
CPU_TEMP_TOPIC = os.getenv("CPU_TEMP_TOPIC")
CPU_USAGE_TOPIC = os.getenv("CPU_USAGE_TOPIC")
WIFI_SIGNAL_TOPIC = os.getenv("WIFI_SIGNAL_TOPIC")
BATTERY_TOPIC = os.getenv("BATTERY_TOPIC")

# 텔레옵 속도 설정
LINEAR_SPEED = float(os.getenv("LINEAR_SPEED"))  # m/s
ANGULAR_SPEED = float(os.getenv("ANGULAR_SPEED"))  # rad/s

# YOLO 인식 설정
YOLO_ENABLED = os.getenv("YOLO_ENABLED", "true").lower() == "true"
YOLO_MODEL_PATH = Path(__file__).parent.parent / "auto_labeling" / "models" / "user_detection" / "weights" / "best.pt"
YOLO_CONFIDENCE = float(os.getenv("YOLO_CONFIDENCE"))

