from fastapi import FastAPI, Request
from fastapi.responses import StreamingResponse, JSONResponse, HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn
import os
from dotenv import load_dotenv

load_dotenv()

# ROS2 관련 import
try:
    import rclpy
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

# src 모듈에서 RobotNode import
from src.ros_node import RobotNode, DummyRobotNode

import cv2
import asyncio
import threading
import numpy as np
import httpx
from datetime import datetime, timedelta
from urllib.parse import quote

# 기상청 API 키
WEATHER_API_KEY = os.getenv("WEATHER_API_KEY", "")


app = FastAPI()

# CORS 설정
HOST = os.getenv("HOST", "0.0.0.0")
PORT = int(os.getenv("PORT", "8000"))

origins = [
    f"http://localhost:{PORT}",
    f"http://127.0.0.1:{PORT}",
    f"http://{HOST}",
    f"http://{HOST}:{PORT}",
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


# 전역 노드 변수
robot_node = None


def init_ros():
    """ROS2 초기화 및 노드 시작"""
    global robot_node
    if not ROS_AVAILABLE:
        print("ROS2 not available, using dummy node.")
        robot_node = DummyRobotNode()
        return

    try:
        rclpy.init()
        robot_node = RobotNode()

        # 별도 스레드에서 ROS2 spin 실행
        def spin_node():
            rclpy.spin(robot_node)

        thread = threading.Thread(target=spin_node, daemon=True)
        thread.start()
        print("ROS2 노드 초기화 완료")
    except Exception as e:
        print(f"ROS2 초기화 실패: {e}")
        robot_node = DummyRobotNode()


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
    """정적 자산에 대한 Cache-Control 헤더 설정"""
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


# ============================================
# API 엔드포인트
# ============================================

@app.get("/api/status")
async def get_robot_status():
    """로봇 상태 정보 (배터리, 모드 등)"""
    robot_status = {
        "battery": 74,
        "mode": "X-BOOST",
        "voltage": 23.8,
        "main_ac": "30 KWH",
    }
    return JSONResponse(content=robot_status)


@app.get("/api/system")
async def get_system_info():
    """시스템 정보 (CPU 온도, 사용률, WiFi 신호)"""
    if robot_node:
        return JSONResponse(content=robot_node.get_system_info())
    return JSONResponse(content={"cpu_temperature": 0.0, "cpu_usage": 0.0, "wifi_signal": 0})


class TeleopCommand(BaseModel):
    """텔레옵 명령 모델"""
    key: str  # w, a, s, d, x


@app.post("/api/teleop")
async def teleop_control(command: TeleopCommand):
    """WASDX 키 입력으로 로봇 제어"""
    if not robot_node:
        return JSONResponse(content={"status": "error", "message": "Robot node not available"}, status_code=503)

    key = command.key.lower()
    
    if key == "w":
        robot_node.move_forward()
    elif key == "a":
        robot_node.turn_left()
    elif key == "s" or key == "stop":
        robot_node.stop()
    elif key == "d":
        robot_node.turn_right()
    elif key == "x":
        robot_node.move_backward()
    else:
        return JSONResponse(content={"status": "error", "message": f"Unknown key: {key}"}, status_code=400)

    return JSONResponse(content={"status": "ok", "command": key})


# 지역별 격자 좌표 (기상청 API용)
LOCATION_GRID = {
    "서울": {"nx": 60, "ny": 127},
    "부산": {"nx": 98, "ny": 76},
    "대구": {"nx": 89, "ny": 90},
    "인천": {"nx": 55, "ny": 124},
    "광주": {"nx": 58, "ny": 74},
    "대전": {"nx": 67, "ny": 100},
    "울산": {"nx": 102, "ny": 84},
    "세종": {"nx": 66, "ny": 103},
    "수원": {"nx": 60, "ny": 121},
}


@app.get("/api/weather")
async def get_weather():
    """기상청 API를 통해 현재 날씨 정보 조회"""
    # 기본 위치: 서울 (나중에 로봇 위치로 변경 가능)
    location = "서울"
    grid = LOCATION_GRID.get(location, LOCATION_GRID["서울"])
    
    # 기상청 API는 매시간 정각에 업데이트됨
    # base_time은 0200, 0500, 0800, 1100, 1400, 1700, 2000, 2300
    now = datetime.now()
    base_times = ["0200", "0500", "0800", "1100", "1400", "1700", "2000", "2300"]
    
    # 현재 시간보다 이전인 가장 최근 base_time 찾기
    current_hour = now.hour
    base_time = "2300"  # 기본값
    base_date = now.strftime("%Y%m%d")
    
    for bt in base_times:
        bt_hour = int(bt[:2])
        if current_hour >= bt_hour + 1:  # API 생성 시간 고려 (+1시간)
            base_time = bt
    
    # 만약 현재 시간이 02시 이전이면 전날 23시 데이터 사용
    if current_hour < 3:
        base_date = (now - timedelta(days=1)).strftime("%Y%m%d")
        base_time = "2300"
    
    try:
        url = "http://apis.data.go.kr/1360000/VilageFcstInfoService_2.0/getUltraSrtNcst"
        params = {
            "serviceKey": WEATHER_API_KEY,
            "numOfRows": "10",
            "pageNo": "1",
            "dataType": "JSON",
            "base_date": base_date,
            "base_time": now.strftime("%H") + "00",  # 초단기실황은 매시간 정각
            "nx": grid["nx"],
            "ny": grid["ny"],
        }
        
        async with httpx.AsyncClient(timeout=10.0) as client:
            response = await client.get(url, params=params)
            data = response.json()
        
        # 응답에서 온도(T1H) 추출
        temperature = None
        if "response" in data and "body" in data["response"]:
            items = data["response"]["body"].get("items", {}).get("item", [])
            for item in items:
                if item.get("category") == "T1H":  # 기온
                    temperature = float(item.get("obsrValue", 0))
                    break
        
        return JSONResponse(content={
            "location": location,
            "temperature": temperature if temperature is not None else 0,
            "timestamp": now.strftime("%Y-%m-%d %H:%M"),
        })
    
    except Exception as e:
        print(f"날씨 API 오류: {e}")
        return JSONResponse(content={
            "location": location,
            "temperature": 0,
            "error": str(e),
        })


# ============================================
# 스트리밍 엔드포인트
# ============================================

@app.get("/camera/stream")
async def camera_stream():
    """터틀봇3 카메라 스트림 (MJPEG)"""

    async def generate():
        while True:
            if robot_node and robot_node.latest_camera_frame is not None:
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
            else:
                # 카메라 대기 화면 생성
                blank_image = np.zeros((240, 320, 3), dtype=np.uint8)
                blank_image[:] = (30, 30, 30)
                cv2.putText(
                    blank_image, "CAMERA", (90, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 107, 53), 2
                )
                cv2.putText(
                    blank_image, "Waiting...", (100, 140),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (139, 142, 152), 2
                )
                ret, buffer = cv2.imencode(".jpg", blank_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
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


@app.get("/slam/map")
async def slam_map():
    """SLAM 지도 스트림"""

    async def generate():
        while True:
            if robot_node and robot_node.latest_map_frame is not None:
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
                blank_image = np.zeros((400, 600, 3), dtype=np.uint8)
                blank_image[:] = (30, 30, 30)
                cv2.putText(
                    blank_image, "SLAM MAP", (180, 180),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 107, 53), 3
                )
                cv2.putText(
                    blank_image, "Waiting for /map topic...", (120, 230),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (139, 142, 152), 2
                )
                ret, buffer = cv2.imencode(".jpg", blank_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ret:
                    frame = buffer.tobytes()
                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                    )
            await asyncio.sleep(0.5)  # 2 FPS

    return StreamingResponse(
        generate(), media_type="multipart/x-mixed-replace; boundary=frame"
    )


# ============================================
# HTML 템플릿 라우트
# ============================================

@app.get("/", response_class=HTMLResponse)
async def read_root(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})


@app.get("/dashboard", response_class=HTMLResponse)
async def read_dashboard(request: Request):
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
    uvicorn.run("main:app", host=HOST, port=PORT, reload=True)
