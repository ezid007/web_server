from fastapi import FastAPI, Request
from fastapi.responses import StreamingResponse, JSONResponse, HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from nav_msgs.msg import OccupancyGrid
    from cv_bridge import CvBridge
    from tf2_ros import Buffer, TransformListener
    from rclpy.time import Time
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    # Dummy classes/imports to prevent NameError if used in type hints or inheritance
    class Node: pass
    class Buffer: pass
    class TransformListener: pass
    class Time: pass

import cv2
import asyncio
import threading
import numpy as np


app = FastAPI()

# CORS 설정
origins = [
    "http://localhost:8000",
    "http://127.0.0.1:8000",
    "http://43.200.211.61",
    "http://43.200.211.61:8000",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 정적 파일 마운트
app.mount("/static", StaticFiles(directory="static"), name="static")

# 템플릿 설정
templates = Jinja2Templates(directory="templates")


if ROS_AVAILABLE:
    # 3. ROS2 통합 노드 클래스 (카메라 + SLAM 지도)
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
                # 값 범위: -1 (unknown), 0 (free), 100 (occupied)
                width = msg.info.width
                height = msg.info.height
                data = np.array(msg.data, dtype=np.int8).reshape((height, width))

                # 시각화를 위한 색상 매핑
                # -1 (unknown) -> 회색(128)
                # 0 (free) -> 흰색(255)
                # 100 (occupied) -> 검은색(0)
                map_image = np.zeros((height, width), dtype=np.uint8)
                map_image[data == -1] = 128  # unknown: 회색
                map_image[data == 0] = 255  # free: 흰색
                map_image[data == 100] = 0  # occupied: 검은색

                # 지도 회전 (상하 반전)
                map_image = cv2.flip(map_image, 0)

                # 컬러로 변환 (3채널)
                map_image_color = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR)

                # 로봇 위치 가져오기 및 표시
                try:
                    transform = self.tf_buffer.lookup_transform(
                        "map",  # 목표 프레임
                        "base_footprint",  # 소스 프레임 (로봇 위치)
                        Time(),  # 최신 시간
                    )

                    # 로봇 위치를 지도 좌표로 변환
                    robot_x = transform.transform.translation.x
                    robot_y = transform.transform.translation.y

                    # 지도 픽셀 좌표로 변환
                    resolution = msg.info.resolution
                    origin_x = msg.info.origin.position.x
                    origin_y = msg.info.origin.position.y

                    pixel_x = int((robot_x - origin_x) / resolution)
                    pixel_y = int((robot_y - origin_y) / resolution)

                    # 상하 반전 적용 (flip 때문에)
                    pixel_y = height - pixel_y

                    # 로봇 위치 표시 (주황색 원)
                    if 0 <= pixel_x < width and 0 <= pixel_y < height:
                        cv2.circle(
                            map_image_color, (pixel_x, pixel_y), 8, (0, 107, 255), -1
                        )  # 주황색 원
                        cv2.circle(
                            map_image_color, (pixel_x, pixel_y), 10, (255, 255, 255), 2
                        )  # 흰색 테두리

                        # 로봇 방향 표시 (화살표)
                        # 쿼터니언에서 yaw 각도 추출
                        quat = transform.transform.rotation
                        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
                        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
                        yaw = np.arctan2(siny_cosp, cosy_cosp)

                        # 화살표 끝점 계산
                        arrow_length = 20
                        arrow_x = int(pixel_x + arrow_length * np.cos(yaw))
                        arrow_y = int(pixel_y - arrow_length * np.sin(yaw))  # y축 반전

                        cv2.arrowedLine(
                            map_image_color,
                            (pixel_x, pixel_y),
                            (arrow_x, arrow_y),
                            (0, 107, 255),  # 주황색
                            3,
                            tipLength=0.3,
                        )

                except Exception as e:
                    self.get_logger().debug(f"로봇 위치 가져오기 실패: {e}")

                # 적절한 크기로 리사이즈 (600x400)
                map_image_resized = cv2.resize(
                    map_image_color, (600, 400), interpolation=cv2.INTER_NEAREST
                )

                self.latest_map_frame = map_image_resized

            except Exception as e:
                self.get_logger().error(f"SLAM 지도 변환 실패: {e}")
else:
    # ROS2가 없을 때 사용할 더미 클래스
    class RobotNode:
        def __init__(self):
            self.latest_camera_frame = None
            self.latest_map_frame = None
        
        def destroy_node(self):
            pass



# 전역 노드 변수
robot_node = None


def init_ros():
    """ROS2 초기화 및 노드 시작"""
    global robot_node
    if not ROS_AVAILABLE:
        print("ROS2 not available, skipping initialization.")
        robot_node = RobotNode() # Dummy node
        return

    try:
        rclpy.init()
        robot_node = RobotNode()

        # 별도 스레드에서 ROS2 spin 실행
        def spin_node():
            rclpy.spin(robot_node)

        thread = threading.Thread(target=spin_node, daemon=True)
        thread.start()
    except Exception as e:
        print(f"ROS2 초기화 실패: {e}")


# FastAPI 시작 시 ROS2 초기화
@app.on_event("startup")
async def startup_event():
    init_ros()


@app.on_event("shutdown")
async def shutdown_event():
    if robot_node:
        robot_node.destroy_node()
    if ROS_AVAILABLE and rclpy.ok():
        rclpy.shutdown()


@app.middleware("http")
async def add_cache_header(request: Request, call_next):
    """정적 자산에 대한 Cache-Control 헤더를 설정하여 클라이언트 측 캐싱을 개선합니다.

    - 이미지: 1년, 불변 (immutable)
    - CSS/JS/폰트: 1주, 불변 (immutable)
    - 비디오: 1주
    - 기타: 1시간
    """
    response = await call_next(request)
    path = request.url.path.lower()
    if path.startswith("/static/"):
        if path.endswith((".jpg", ".jpeg", ".png", ".webp", ".svg", ".gif")):
            response.headers["Cache-Control"] = "public, max-age=31536000, immutable"
        elif path.endswith((".css", ".js", ".woff", ".woff2", ".ttf", ".otf")):
            response.headers["Cache-Control"] = "public, max-age=604800, immutable"
        elif path.endswith(".mp4"):
            response.headers["Cache-Control"] = "public, max-age=604800"
        elif path.endswith(".json"):
            response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
        else:
            response.headers["Cache-Control"] = "public, max-age=3600"
    return response


# [API] 로봇 상태 데이터 제공
@app.get("/api/status")
async def get_robot_status():
    # 나중에 실제 로봇 데이터(배터리, 속도 등)를 여기서 넘겨줍니다.
    robot_status = {
        "battery": 74,
        "mode": "X-BOOST",
        "voltage": 23.8,
        "main_ac": "30 KWH",
    }
    return JSONResponse(content=robot_status)


# [라우터 5] 카메라 스트림 엔드포인트 (MJPEG)
@app.get("/camera/stream")
async def camera_stream():
    """터틀봇3 카메라 스트림을 MJPEG 형식으로 제공"""

    async def generate():
        while True:
            if robot_node and robot_node.latest_camera_frame is not None:
                # OpenCV 이미지를 JPEG로 인코딩
                ret, buffer = cv2.imencode(
                    ".jpg",
                    robot_node.latest_camera_frame,
                    [cv2.IMWRITE_JPEG_QUALITY, 80],
                )
                if ret:
                    frame = buffer.tobytes()
                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                    )
            await asyncio.sleep(1 / 30)  # 30 FPS

    return StreamingResponse(
        generate(), media_type="multipart/x-mixed-replace; boundary=frame"
    )


# [라우터 6] SLAM 지도 스트림 엔드포인트
@app.get("/slam/map")
async def slam_map():
    """SLAM 지도 스트림 (Cartographer /map 토픽에서 실시간 수신)"""

    async def generate():
        while True:
            if robot_node and robot_node.latest_map_frame is not None:
                # SLAM 지도를 JPEG로 인코딩
                ret, buffer = cv2.imencode(
                    ".jpg", robot_node.latest_map_frame, [cv2.IMWRITE_JPEG_QUALITY, 85]
                )
                if ret:
                    frame = buffer.tobytes()
                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                    )
            else:
                # 지도가 없을 때 임시 이미지 생성
                blank_image = np.zeros((400, 600, 3), dtype=np.uint8)
                blank_image[:] = (30, 30, 30)  # 어두운 회색

                # 중앙에 "SLAM MAP - Waiting..." 텍스트
                cv2.putText(
                    blank_image,
                    "SLAM MAP",
                    (180, 180),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.5,
                    (255, 107, 53),
                    3,
                )
                cv2.putText(
                    blank_image,
                    "Waiting for /map topic...",
                    (120, 230),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (139, 142, 152),
                    2,
                )

                ret, buffer = cv2.imencode(
                    ".jpg", blank_image, [cv2.IMWRITE_JPEG_QUALITY, 80]
                )
                if ret:
                    frame = buffer.tobytes()
                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                    )

            await asyncio.sleep(0.5)  # 2 FPS (지도는 자주 업데이트 안됨)

    return StreamingResponse(
        generate(), media_type="multipart/x-mixed-replace; boundary=frame"
    )


# HTML 템플릿 라우트
@app.get("/", response_class=HTMLResponse)
async def read_root(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})


@app.get("/dashboard", response_class=HTMLResponse)
async def read_dashboard(request: Request):
    # 초기 로봇 상태 데이터 (템플릿 렌더링용)
    robot_status = {
        "battery": 74,
        "mode": "X-BOOST",
        "voltage": 23.8,
        "main_ac": "30 KWH",
    }
    return templates.TemplateResponse(
        "dashboard.html", {"request": request, "status": robot_status}
    )


@app.get("/works", response_class=HTMLResponse)
async def read_works(request: Request):
    return templates.TemplateResponse("works.html", {"request": request})


@app.get("/contact", response_class=HTMLResponse)
async def read_contact(request: Request):
    return templates.TemplateResponse("contact.html", {"request": request})


if __name__ == "__main__":
    # 로컬 테스트용 (8000번 포트)
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
