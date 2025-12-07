"""
ROS2 통합 노드 - 카메라, SLAM, 시스템 정보 구독 및 텔레옵 발행
"""
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from sensor_msgs.msg import Image, BatteryState
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import Twist
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
from . import config


class RobotNode(Node if ROS_AVAILABLE else object):
    """ROS2 통합 노드 - 모든 구독/발행 기능 통합"""

    def __init__(self):
        if ROS_AVAILABLE:
            super().__init__("robot_dashboard_node")
            self.bridge = CvBridge()
        
        # 카메라 및 지도 프레임
        self.latest_camera_frame = None
        self.latest_map_frame = None
        self.latest_map_info = None
        self.robot_pose = None

        # 시스템 정보
        self.cpu_temperature = 0.0
        self.cpu_usage = 0.0
        self.wifi_signal = 0
        self.battery_percentage = 0.0
        self.battery_voltage = 0.0

        if ROS_AVAILABLE:
            # TF 버퍼 및 리스너 설정
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            # 카메라 구독
            self.camera_subscription = self.create_subscription(
                Image,
                config.CAMERA_TOPIC,
                self.camera_callback,
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

            self.get_logger().info("RobotNode 초기화 완료")

    def camera_callback(self, msg):
        """카메라 이미지 콜백"""
        try:
            if msg.encoding == "rgb8":
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            if cv_image.shape[1] != 320 or cv_image.shape[0] != 240:
                cv_image = cv2.resize(cv_image, (320, 240))

            self.latest_camera_frame = cv_image
        except Exception as e:
            if ROS_AVAILABLE:
                self.get_logger().error(
                    f"카메라 이미지 변환 실패: {e} (encoding: {msg.encoding})"
                )

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
            map_image[data == 0] = 255   # free: 흰색
            map_image[data == 100] = 0   # occupied: 검은색

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
                    cv2.circle(map_image_color, (pixel_x, pixel_y), 8, (0, 107, 255), -1)
                    cv2.circle(map_image_color, (pixel_x, pixel_y), 10, (255, 255, 255), 2)

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
