from fastapi import FastAPI, Request
from fastapi.responses import StreamingResponse, JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
import cv2
import asyncio
import threading
import numpy as np
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

app = FastAPI()

# CORS 설정
origins = [
    "http://localhost:8000",
    "http://127.0.0.1:8000",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 정적 파일 및 템플릿 설정
app.mount("/static", StaticFiles(directory="static"), name="static")
templates = Jinja2Templates(directory="templates")


# HTML 페이지 라우트
@app.get("/")
async def read_index(request: Request):
    """메인 페이지 (index.html)"""
    return templates.TemplateResponse("index.html", {"request": request})


@app.get("/works")
async def read_works(request: Request):
    """작업물 페이지 (works.html)"""
    return templates.TemplateResponse("works.html", {"request": request})


@app.get("/contact")
async def read_contact(request: Request):
    """문의하기 페이지 (contact.html)"""
    return templates.TemplateResponse("contact.html", {"request": request})


@app.get("/works")
async def read_history(request: Request):
    """작업물 페이지 (works.html)"""
    return templates.TemplateResponse("history.html", {"request": request})


@app.get("/dashboard")
async def read_dashboard(request: Request):
    """대시보드 페이지 (dashboard.html)"""
    # 기본 상태 정보 제공
    status = {
        "battery": 85,
        "mode": "STANDBY",
        "voltage": 12.4,
        "main_ac": "150 KWH",
    }
    return templates.TemplateResponse(
        "dashboard.html", {"request": request, "status": status}
    )


# ROS2 통합 노드 클래스 (카메라 + SLAM 지도)
class RobotNode(Node):
    def __init__(self):
        super().__init__("robot_dashboard_subscriber")
        self.bridge = CvBridge()
        self.latest_camera_frame = None
        self.latest_map_frame = None
        self.latest_map_info = None  # 지도 메타 정보 저장
        self.robot_pose = None  # 로봇 위치 저장

        # TF 버퍼 및 리스너 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 카메라 토픽 구독
        self.camera_subscription = self.create_subscription(
            Image,
            "/camera/image_raw",  # TurtleBot3 카메라 토픽
            self.camera_callback,
            10,
        )

        # SLAM 지도 토픽 구독
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            "/map",  # Cartographer가 발행하는 지도 토픽
            self.map_callback,
            10,
        )

    def camera_callback(self, msg):
        """카메라 이미지 콜백"""
        try:
            # RGB888 포맷으로 들어오는 이미지를 BGR8로 변환
            if msg.encoding == "rgb8":
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # 320x240 크기 확인
            if cv_image.shape[1] != 320 or cv_image.shape[0] != 240:
                cv_image = cv2.resize(cv_image, (320, 240))

            self.latest_camera_frame = cv_image
        except Exception as e:
            self.get_logger().error(
                f"카메라 이미지 변환 실패: {e} (encoding: {msg.encoding})"
            )

    def map_callback(self, msg):
        """SLAM 지도 콜백 - OccupancyGrid를 이미지로 변환"""
        try:
            # 지도 메타 정보 저장
            self.latest_map_info = msg.info

            # OccupancyGrid 데이터를 numpy 배열로 변환
            map_data = np.array(msg.data, dtype=np.int8).reshape(
                (msg.info.height, msg.info.width)
            )

            # 지도 이미지 생성 (장애물: 검정, 빈 공간: 흰색, 미지의 영역: 회색)
            map_image = np.zeros((msg.info.height, msg.info.width), dtype=np.uint8)
            map_image[map_data == -1] = 127  # 미지의 영역 (회색)
            map_image[map_data == 0] = 255  # 빈 공간 (흰색)
            map_image[map_data > 50] = 0  # 장애물 (검정)

            # 로봇 위치 가져오기 및 그리기
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map",
                    "base_link",
                    Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1),
                )
                robot_x = transform.transform.translation.x
                robot_y = transform.transform.translation.y

                # 로봇 위치를 픽셀 좌표로 변환
                pixel_x = int(
                    (robot_x - msg.info.origin.position.x) / msg.info.resolution
                )
                pixel_y = int(
                    (robot_y - msg.info.origin.position.y) / msg.info.resolution
                )

                # 지도 좌표계는 y축이 반대이므로 flip
                pixel_y = msg.info.height - pixel_y

                # 로봇 위치 저장
                self.robot_pose = (pixel_x, pixel_y)

                # BGR로 변환 후 로봇 위치에 빨간 원 그리기
                map_image_bgr = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR)
                cv2.circle(map_image_bgr, (pixel_x, pixel_y), 5, (0, 0, 255), -1)

                self.latest_map_frame = map_image_bgr
            except Exception as tf_error:
                # TF 조회 실패 시 로봇 없이 지도만 표시
                self.latest_map_frame = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR)
                self.get_logger().debug(f"TF 조회 실패: {tf_error}")

        except Exception as e:
            self.get_logger().error(f"지도 변환 실패: {e}")


# 전역 ROS2 노드
robot_node = None


def ros2_spin_thread():
    """ROS2 스핀을 별도 스레드에서 실행"""
    rclpy.spin(robot_node)


@app.on_event("startup")
async def startup_event():
    """FastAPI 시작 시 ROS2 노드 초기화"""
    global robot_node
    rclpy.init()
    robot_node = RobotNode()
    threading.Thread(target=ros2_spin_thread, daemon=True).start()


@app.on_event("shutdown")
def shutdown_event():
    """FastAPI 종료 시 ROS2 정리"""
    if robot_node:
        robot_node.destroy_node()
    rclpy.shutdown()


# 카메라 스트리밍 엔드포인트
@app.get("/camera/stream")
async def camera_stream():
    """카메라 영상을 MJPEG 스트림으로 제공"""

    async def generate():
        while True:
            if robot_node and robot_node.latest_camera_frame is not None:
                # OpenCV 이미지를 JPEG로 인코딩
                _, buffer = cv2.imencode(".jpg", robot_node.latest_camera_frame)
                frame = buffer.tobytes()

                # MJPEG 형식으로 반환
                yield (
                    b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                )
            await asyncio.sleep(0.03)  # 약 30 FPS

    return StreamingResponse(
        generate(), media_type="multipart/x-mixed-replace; boundary=frame"
    )


# SLAM 지도 스트리밍 엔드포인트
@app.get("/map/stream")
async def map_stream():
    """SLAM 지도를 MJPEG 스트림으로 제공"""

    async def generate():
        while True:
            if robot_node and robot_node.latest_map_frame is not None:
                # OpenCV 이미지를 JPEG로 인코딩
                _, buffer = cv2.imencode(".jpg", robot_node.latest_map_frame)
                frame = buffer.tobytes()

                # MJPEG 형식으로 반환
                yield (
                    b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                )
            else:
                # 지도가 없을 때 대기 이미지 생성 (높이, 너비, 채널)
                blank_image = np.zeros((400, 600, 3), dtype=np.uint8)
                blank_image[:] = (30, 30, 30)  # 어두운 회색

                # 중앙에 "SLAM MAP - Waiting..." 텍스트
                cv2.putText(
                    blank_image,
                    "SLAM MAP",
                    (180, 180),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.5,
                    (255, 107, 53),  # 주황색
                    3,
                )
                cv2.putText(
                    blank_image,
                    "Waiting for /map topic...",
                    (120, 230),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (139, 142, 152),  # 회색
                    2,
                )

                _, buffer = cv2.imencode(".jpg", blank_image)
                frame = buffer.tobytes()

                yield (
                    b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                )

            await asyncio.sleep(0.5)  # 2 FPS (지도는 자주 업데이트 안됨)

    return StreamingResponse(
        generate(), media_type="multipart/x-mixed-replace; boundary=frame"
    )


# 로봇 상태 정보 엔드포인트
@app.get("/robot/status")
async def robot_status():
    """로봇의 현재 상태 정보 반환"""
    if not robot_node:
        return JSONResponse(
            content={"error": "ROS2 노드가 초기화되지 않았습니다."}, status_code=503
        )

    status = {
        "battery": 85,  # 예시 값 (실제로는 배터리 토픽에서 가져와야 함)
        "mode": "MAPPING",
        "voltage": 12.4,
        "main_ac": "150 KWH",
        "camera_active": robot_node.latest_camera_frame is not None,
        "map_active": robot_node.latest_map_frame is not None,
    }

    # 로봇 위치 추가
    if robot_node.robot_pose:
        status["robot_position"] = {
            "x": robot_node.robot_pose[0],
            "y": robot_node.robot_pose[1],
        }

    return JSONResponse(content=status)


# 서버 실행
if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
