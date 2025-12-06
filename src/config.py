"""
ROS2 토픽 및 설정값 관리
하드코딩 제거를 위해 모든 설정을 이 파일에서 관리
"""
import os
from dotenv import load_dotenv

load_dotenv()

# 카메라 토픽
CAMERA_TOPIC = os.getenv("CAMERA_TOPIC", "/camera_node/image_raw")

# SLAM 지도 토픽
MAP_TOPIC = os.getenv("MAP_TOPIC", "/map")

# 텔레옵 명령 토픽
CMD_VEL_TOPIC = os.getenv("CMD_VEL_TOPIC", "/cmd_vel")

# 시스템 정보 토픽
CPU_TEMP_TOPIC = os.getenv("CPU_TEMP_TOPIC", "/system/cpu_temperature")
CPU_USAGE_TOPIC = os.getenv("CPU_USAGE_TOPIC", "/system/cpu_usage")
WIFI_SIGNAL_TOPIC = os.getenv("WIFI_SIGNAL_TOPIC", "/system/wifi_signal")
BATTERY_TOPIC = os.getenv("BATTERY_TOPIC", "/battery_state")

# 텔레옵 속도 설정
LINEAR_SPEED = float(os.getenv("LINEAR_SPEED", "0.2"))  # m/s
ANGULAR_SPEED = float(os.getenv("ANGULAR_SPEED", "0.5"))  # rad/s
