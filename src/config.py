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
# 압축 카메라 토픽 (image_transport 사용)
CAMERA_COMPRESSED_TOPIC = os.getenv("CAMERA_COMPRESSED_TOPIC", "/camera_node/image_raw/compressed")

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

# YOLO 모델 경로 (models/ 폴더에서 중앙 관리)
# .env에서 파일명만 적으면 자동으로 models/ 경로가 붙음
# 예: YOLO_MODEL_PATH=user_detection.pt → models/user_detection.pt
_base_dir = Path(__file__).parent.parent
_models_dir = _base_dir / "models"

def _resolve_model_path(model_name: str, default: str) -> Path:
    """모델 경로 해석: 파일명만 있으면 models/ 폴더에서 찾음"""
    path = Path(model_name) if model_name else Path(default)
    # 절대 경로면 그대로 사용
    if path.is_absolute():
        return path
    # 파일명만 있으면 (경로 구분자 없으면) models/ 폴더에서 찾음
    if path.parent == Path("."):
        return _models_dir / path
    # 상대 경로면 base_dir 기준으로 변환
    return _base_dir / path

YOLO_MODEL_PATH = _resolve_model_path(
    os.getenv("YOLO_DASHBOARD_MODEL", ""), "my_yolo.pt"
)
YOLO_LABELING_MODEL = _resolve_model_path(
    os.getenv("YOLO_LABELING_MODEL", ""), "yolo11n.pt"
)

YOLO_CONFIDENCE = float(os.getenv("YOLO_CONFIDENCE", "0.5"))

