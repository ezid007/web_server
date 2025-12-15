"""
ROS2 통합 노드 - 카메라, SLAM, 시스템 정보 구독 및 텔레옵 발행
"""

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from sensor_msgs.msg import Image, CompressedImage, BatteryState, LaserScan
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import Twist, PoseStamped
    from std_msgs.msg import Float32, Int32
    from cv_bridge import CvBridge
    from tf2_ros import Buffer, TransformListener
    from rclpy.time import Time

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

    class Node:
        pass

    class Buffer:
        pass

    class TransformListener:
        pass

    class Time:
        pass


import cv2
import numpy as np
import threading
import queue
import yaml
from pathlib import Path
from . import config

# YOLO 감지기 import (웹에서 ON/OFF 제어용)
try:
    from detect.yolo_detector import yolo_detector
except ImportError:
    yolo_detector = None


# 클래스 설정 파일 경로
CLASSES_FILE = Path(__file__).parent.parent / "auto_labeling" / "classes.yaml"


def load_yolo_classes() -> dict:
    """
    classes.yaml에서 클래스 목록을 읽어와 {id: name} 형태로 반환
    클래스 이름은 이 파일 하나에서만 관리됨
    """
    if CLASSES_FILE.exists():
        try:
            with open(CLASSES_FILE, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                names = data.get('names', ['unknown'])
                return {i: name for i, name in enumerate(names)}
        except Exception as e:
            print(f"⚠️ classes.yaml 로드 실패: {e}")
    return {0: "unknown"}


class RobotNode(Node if ROS_AVAILABLE else object):
    """ROS2 통합 노드 - 모든 구독/발행 기능 통합"""

    def __init__(self):
        if ROS_AVAILABLE:
            super().__init__("robot_dashboard_node")
            self.bridge = CvBridge()

        # 카메라 및 지도 프레임
        self.latest_camera_frame = None
        self.latest_raw_frame = None  # 원본 프레임 (YOLO 없음, 오토 라벨링용)
        self.latest_map_frame = None
        self.latest_map_info = None
        self.robot_pose = None

        # 시스템 정보
        self.cpu_temperature = 0.0
        self.cpu_usage = 0.0
        self.wifi_signal = 0
        self.battery_percentage = 0.0
        self.battery_voltage = 0.0

        # Lidar data (for pet mode)
        self.latest_scan = None

        # YOLO 비동기 처리를 위한 큐와 스레드
        self.yolo_queue = queue.Queue(maxsize=1)  # 최신 프레임만 유지
        self.yolo_thread = None
        self.yolo_running = False
        self.yolo_classes = load_yolo_classes()  # classes.yaml에서 로드

        # YOLO 워커 스레드 시작 (웹에서 토글 시 lazy load)
        self.yolo_running = True
        self.yolo_thread = threading.Thread(
            target=self._yolo_worker, daemon=True
        )
        self.yolo_thread.start()
        print("✅ YOLO 비동기 워커 시작됨 (웹 토글로 제어)")

        if ROS_AVAILABLE:
            # TF 버퍼 및 리스너 설정
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            # 카메라 구독 (압축 이미지)
            self.camera_subscription = self.create_subscription(
                CompressedImage,
                config.CAMERA_COMPRESSED_TOPIC,
                self.compressed_camera_callback,
                10,
            )

            # SLAM 지도 구독 - transient_local QoS (map_server의 latch된 메시지 수신)
            map_qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self.map_subscription = self.create_subscription(
                OccupancyGrid,
                config.MAP_TOPIC,
                self.map_callback,
                map_qos,
            )

            # 시스템 정보 구독
            self.cpu_temp_subscription = self.create_subscription(
                Float32,
                config.CPU_TEMP_TOPIC,
                self.cpu_temp_callback,
                10,
            )

            self.cpu_usage_subscription = self.create_subscription(
                Float32,
                config.CPU_USAGE_TOPIC,
                self.cpu_usage_callback,
                10,
            )

            self.wifi_signal_subscription = self.create_subscription(
                Int32,
                config.WIFI_SIGNAL_TOPIC,
                self.wifi_signal_callback,
                10,
            )

            # 배터리 상태 구독
            self.battery_subscription = self.create_subscription(
                BatteryState,
                config.BATTERY_TOPIC,
                self.battery_callback,
                10,
            )

            # 텔레옵 퍼블리셔
            self.cmd_vel_publisher = self.create_publisher(
                Twist,
                config.CMD_VEL_TOPIC,
                10,
            )

            # Nav2 Goal 퍼블리셔 (/goal_pose)
            self.goal_publisher = self.create_publisher(
                PoseStamped,
                "/goal_pose",
                10,
            )

            # Lidar 구독 (pet mode 거리 측정용)
            self.lidar_subscription = self.create_subscription(
                LaserScan,
                config.LIDAR_TOPIC,
                self.lidar_callback,
                10,
            )

            self.get_logger().info("RobotNode 초기화 완료")

    def _yolo_worker(self):
        """
        YOLO 추론을 별도 스레드에서 비동기로 처리
        yolo_detector를 직접 사용하여 웹 토글과 연동
        """
        while self.yolo_running:
            try:
                # 큐에서 프레임 가져오기 (타임아웃 0.1초)
                frame = self.yolo_queue.get(timeout=0.1)

                # yolo_detector를 사용하여 감지 (웹 토글로 ON/OFF)
                if yolo_detector and yolo_detector.enabled:
                    detections = yolo_detector.detect_persons(frame)
                    if detections:
                        result_frame = self._draw_detections(frame, detections)
                    else:
                        result_frame = frame
                else:
                    result_frame = frame
                
                self.latest_camera_frame = result_frame

            except queue.Empty:
                continue
            except Exception as e:
                print(f"⚠️ YOLO 워커 오류: {e}")

    def compressed_camera_callback(self, msg: CompressedImage):
        """압축된 카메라 이미지 콜백 - JPEG 압축 해제"""
        try:
            # JPEG 압축 해제
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is None:
                if ROS_AVAILABLE:
                    self.get_logger().error("JPEG 디코딩 실패")
                return

            # 640x480으로 리사이즈
            if cv_image.shape[1] != 640 or cv_image.shape[0] != 480:
                cv_image = cv2.resize(cv_image, (640, 480))

            # 원본 프레임 저장 (오토 라벨링용 - YOLO 없음)
            self.latest_raw_frame = cv_image.copy()

            # YOLO 워커에 프레임 전달 (큐에 넣기)
            if self.yolo_running:
                # 큐가 가득 차면 이전 프레임 버리고 새 프레임 넣기
                try:
                    self.yolo_queue.get_nowait()
                except queue.Empty:
                    pass
                self.yolo_queue.put(cv_image.copy())

                # YOLO 결과가 아직 없으면 원본 표시
                if self.latest_camera_frame is None:
                    self.latest_camera_frame = cv_image
            else:
                # YOLO 워커 비활성화 시 원본 그대로 표시
                self.latest_camera_frame = cv_image
        except Exception as e:
            if ROS_AVAILABLE:
                self.get_logger().error(
                    f"압축 이미지 디코딩 실패: {e}"
                )

    def _draw_detections(self, frame, detections: list):
        """
        YOLO 감지 결과를 프레임에 그리기
        yolo_detector의 결과를 사용하며, 클래스 이름을 매핑
        """
        try:
            for det in detections:
                x1, y1, x2, y2 = det["bbox"]
                conf = det["confidence"]
                class_id = det.get("class_id", 0)

                # 클래스 이름 결정 (classes.yaml 기반)
                if class_id == 0:
                    label = "Person"  # OpenCV는 한글 미지원
                    color = (128, 128, 128)  # 회색 - 미학습
                else:
                    label = self.yolo_classes.get(class_id, f"ID:{class_id}")
                    color = (0, 255, 0)  # 초록 - 학습된 사람

                # 바운딩 박스 그리기
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

                # 라벨 배경
                label_text = f"{label} {conf:.0%}"
                (text_w, text_h), _ = cv2.getTextSize(
                    label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
                )
                cv2.rectangle(
                    frame, (x1, y1 - text_h - 10), (x1 + text_w + 4, y1), color, -1
                )

                # 라벨 텍스트
                cv2.putText(
                    frame,
                    label_text,
                    (x1 + 2, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2,
                )
        except Exception as e:
            print(f"⚠️ YOLO 그리기 오류: {e}")

        return frame

    def map_callback(self, msg):
        """SLAM 지도 콜백 - OccupancyGrid를 이미지로 변환"""
        try:
            self.get_logger().debug(f"맵 콜백 수신: {msg.info.width}x{msg.info.height}")
            self.latest_map_info = msg.info

            width = msg.info.width
            height = msg.info.height
            data = np.array(msg.data, dtype=np.int8).reshape((height, width))

            map_image = np.zeros((height, width), dtype=np.uint8)
            map_image[data == -1] = 128  # unknown: 회색
            map_image[data == 0] = 255  # free: 흰색
            map_image[data == 100] = 0  # occupied: 검은색

            map_image = cv2.flip(map_image, 0)
            map_image_color = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR)

            # 로봇 위치 표시
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map", "base_footprint", Time()
                )

                robot_x = transform.transform.translation.x
                robot_y = transform.transform.translation.y

                resolution = msg.info.resolution
                origin_x = msg.info.origin.position.x
                origin_y = msg.info.origin.position.y

                pixel_x = int((robot_x - origin_x) / resolution)
                pixel_y = int((robot_y - origin_y) / resolution)
                pixel_y = height - pixel_y

                if 0 <= pixel_x < width and 0 <= pixel_y < height:
                    cv2.circle(
                        map_image_color, (pixel_x, pixel_y), 8, (0, 107, 255), -1
                    )
                    cv2.circle(
                        map_image_color, (pixel_x, pixel_y), 10, (255, 255, 255), 2
                    )

                    # 로봇 방향 화살표
                    quat = transform.transform.rotation
                    siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
                    cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
                    yaw = np.arctan2(siny_cosp, cosy_cosp)

                    arrow_length = 20
                    arrow_x = int(pixel_x + arrow_length * np.cos(yaw))
                    arrow_y = int(pixel_y - arrow_length * np.sin(yaw))

                    cv2.arrowedLine(
                        map_image_color,
                        (pixel_x, pixel_y),
                        (arrow_x, arrow_y),
                        (0, 107, 255),
                        3,
                        tipLength=0.3,
                    )

            except Exception as e:
                if ROS_AVAILABLE:
                    self.get_logger().debug(f"로봇 위치 가져오기 실패: {e}")

            map_image_resized = cv2.resize(
                map_image_color, (600, 400), interpolation=cv2.INTER_NEAREST
            )

            self.latest_map_frame = map_image_resized

        except Exception as e:
            if ROS_AVAILABLE:
                self.get_logger().error(f"SLAM 지도 변환 실패: {e}")

    def cpu_temp_callback(self, msg):
        """CPU 온도 콜백"""
        self.cpu_temperature = msg.data

    def cpu_usage_callback(self, msg):
        """CPU 사용률 콜백"""
        self.cpu_usage = msg.data

    def wifi_signal_callback(self, msg):
        """WiFi 신호 강도 콜백"""
        self.wifi_signal = msg.data

    def battery_callback(self, msg):
        """배터리 상태 콜백"""
        self.battery_percentage = msg.percentage  # 이미 0-100% 범위
        self.battery_voltage = msg.voltage

    def lidar_callback(self, msg):
        """Lidar scan 콜백 (pet mode용)"""
        self.latest_scan = msg

    def publish_cmd_vel(self, linear_x: float, angular_z: float):
        """텔레옵 명령 발행"""
        if not ROS_AVAILABLE:
            return

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_publisher.publish(twist)

    def move_forward(self):
        """전진"""
        self.publish_cmd_vel(config.LINEAR_SPEED, 0.0)

    def move_backward(self):
        """후진"""
        self.publish_cmd_vel(-config.LINEAR_SPEED, 0.0)

    def turn_left(self):
        """좌회전"""
        self.publish_cmd_vel(0.0, config.ANGULAR_SPEED)

    def turn_right(self):
        """우회전"""
        self.publish_cmd_vel(0.0, -config.ANGULAR_SPEED)

    def stop(self):
        """정지"""
        self.publish_cmd_vel(0.0, 0.0)

    def get_system_info(self) -> dict:
        """시스템 정보 반환"""
        return {
            "cpu_temperature": round(self.cpu_temperature, 1),
            "cpu_usage": round(self.cpu_usage, 1),
            "wifi_signal": self.wifi_signal,
            "battery_percentage": round(self.battery_percentage, 1),
            "battery_voltage": round(self.battery_voltage, 2),
        }

    def send_nav_goal(self, x: float, y: float) -> bool:
        """
        Nav2 Goal 전송 - 지정된 좌표로 네비게이션 Goal 발행
        
        Args:
            x: 목표 X 좌표 (m)
            y: 목표 Y 좌표 (m)
            
        Returns:
            bool: 전송 성공 여부
        """
        if not ROS_AVAILABLE:
            print(f"🚧 [DummyNode] Nav2 Goal: ({x}, {y})")
            return False

        try:
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = 0.0
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 1.0

            self.goal_publisher.publish(goal)
            self.get_logger().info(f"🎯 Nav2 Goal 전송: ({x:.2f}, {y:.2f})")
            return True
        except Exception as e:
            self.get_logger().error(f"❌ Nav2 Goal 전송 실패: {e}")
            return False


# 더미 노드 (ROS2가 없을 때 사용)
class DummyRobotNode:
    """ROS2가 없을 때 사용할 더미 노드"""

    def __init__(self):
        self.latest_camera_frame = None
        self.latest_map_frame = None
        self.cpu_temperature = 0.0
        self.cpu_usage = 0.0
        self.wifi_signal = 0
        self.battery_percentage = 0.0
        self.battery_voltage = 0.0
        self.latest_raw_frame = None  # 원본 프레임 (YOLO 없음)

        # YOLO 모델 로드 (ROS2 없이도 테스트용)
        self.yolo_model = None
        self.yolo_classes = load_yolo_classes()  # classes.yaml에서 로드
        if config.YOLO_ENABLED and config.YOLO_MODEL_PATH.exists():
            try:
                from ultralytics import YOLO

                self.yolo_model = YOLO(str(config.YOLO_MODEL_PATH))
                print(f"✅ YOLO 모델 로드됨 (DummyNode): {config.YOLO_MODEL_PATH}")
            except Exception as e:
                print(f"⚠️ YOLO 모델 로드 실패: {e}")

    def destroy_node(self):
        pass

    def move_forward(self):
        pass

    def move_backward(self):
        pass

    def turn_left(self):
        pass

    def turn_right(self):
        pass

    def stop(self):
        pass

    def get_system_info(self) -> dict:
        return {
            "cpu_temperature": 0.0,
            "cpu_usage": 0.0,
            "wifi_signal": 0,
            "battery_percentage": 0.0,
            "battery_voltage": 0.0,
        }

    def send_nav_goal(self, x: float, y: float) -> bool:
        """더미 Nav2 Goal 전송 (ROS2 없을 때)"""
        print(f"🚧 [DummyNode] Nav2 Goal: ({x:.2f}, {y:.2f})")
        return True  # 테스트용으로 성공 반환

    def publish_cmd_vel(self, linear_x: float, angular_z: float):
        """더미 cmd_vel 발행"""
        pass
