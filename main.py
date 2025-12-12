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
from src.chatbot import router as chatbot_router, load_chatbot_model, unload_chatbot_model
from src.location import get_location_from_ip_async

import cv2
import asyncio
import threading
import numpy as np
import httpx
from datetime import datetime, timedelta
from urllib.parse import quote

# 감시 모드 모듈
from detect.yolo_detector import yolo_detector
from detect.surveillance import surveillance_system

# 펫 모드 모듈
from my_pet.pet_mode import pet_mode_system

# 기상청 API 키
WEATHER_API_KEY = os.getenv("WEATHER_API_KEY")


app = FastAPI()

# CORS 설정
HOST = os.getenv("HOST")
PORT = int(os.getenv("PORT"))

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
    
    # 감시 모드 콜백 설정
    def send_nav_goal(x, y):
        if robot_node and hasattr(robot_node, 'send_nav_goal'):
            return robot_node.send_nav_goal(x, y)
        return False
    
    def get_robot_frame():
        if robot_node and robot_node.latest_camera_frame is not None:
            return robot_node.latest_camera_frame.copy()
        return None
    
    def get_cctv_frame():
        # CCTV 프레임 가져오기 (외부 서버에서)
        # TODO: CCTV 서버에서 프레임 가져오기 구현
        return None
    
    surveillance_system.set_callbacks(
        send_nav_goal=send_nav_goal,
        get_robot_frame=get_robot_frame,
        get_cctv_frame=get_cctv_frame
    )
    print("✅ 감시 모드 콜백 설정 완료")
    
    # 펫 모드 콜백 설정
    def get_camera_frame():
        if robot_node and robot_node.latest_camera_frame is not None:
            return robot_node.latest_camera_frame.copy()
        return None
    
    def get_lidar_scan():
        if robot_node and robot_node.latest_scan is not None:
            return robot_node.latest_scan
        return None
    
    def send_cmd_vel(linear, angular):
        if robot_node:
            robot_node.publish_cmd_vel(linear, angular)
    
    pet_mode_system.set_callbacks(
        get_camera_frame=get_camera_frame,
        get_lidar_scan=get_lidar_scan,
        send_nav_goal=send_nav_goal,
        send_cmd_vel=send_cmd_vel
    )
    print("✅ 펫 모드 콜백 설정 완료")
    
    # PHi-4 챗봇 모델 로드
    await load_chatbot_model()


@app.on_event("shutdown")
async def shutdown_event():
    if robot_node:
        robot_node.destroy_node()
    if ROS_AVAILABLE and rclpy.ok():
        rclpy.shutdown()
    # PHi-4 챗봇 모델 언로드
    await unload_chatbot_model()


# 챗봇 라우터 등록
app.include_router(chatbot_router)


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
# Favicon
# ============================================
from fastapi.responses import FileResponse

@app.get("/favicon.ico", include_in_schema=False)
async def favicon():
    """브라우저 favicon 요청 처리"""
    return FileResponse("static/images/logo-inverse.svg", media_type="image/svg+xml")


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


# 지역별 격자 좌표 (기상청 API용) - 위경도 변환 실패 시 폴백용
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

# 하늘 상태 코드
SKY_STATUS = {
    "1": {"name": "맑음", "icon": "fa-sun", "color": "#FFD700"},
    "3": {"name": "구름많음", "icon": "fa-cloud-sun", "color": "#87CEEB"},
    "4": {"name": "흐림", "icon": "fa-cloud", "color": "#A9A9A9"},
}

# 강수 형태 코드
PTY_STATUS = {
    "0": {"name": "없음", "icon": None},
    "1": {"name": "비", "icon": "fa-cloud-rain"},
    "2": {"name": "비/눈", "icon": "fa-cloud-meatball"},
    "3": {"name": "눈", "icon": "fa-snowflake"},
    "4": {"name": "소나기", "icon": "fa-cloud-showers-heavy"},
    "5": {"name": "빗방울", "icon": "fa-cloud-rain"},
    "6": {"name": "빗방울눈날림", "icon": "fa-cloud-meatball"},
    "7": {"name": "눈날림", "icon": "fa-snowflake"},
}


def latlon_to_grid(lat: float, lon: float) -> dict:
    """위도/경도를 기상청 격자 좌표로 변환 (LCC DFS 좌표 변환)"""
    import math
    
    RE = 6371.00877  # 지구 반경(km)
    GRID = 5.0  # 격자 간격(km)
    SLAT1 = 30.0  # 투영 위도1(degree)
    SLAT2 = 60.0  # 투영 위도2(degree)
    OLON = 126.0  # 기준점 경도(degree)
    OLAT = 38.0  # 기준점 위도(degree)
    XO = 43  # 기준점 X좌표(GRID)
    YO = 136  # 기준점 Y좌표(GRID)
    
    DEGRAD = math.pi / 180.0
    
    re = RE / GRID
    slat1 = SLAT1 * DEGRAD
    slat2 = SLAT2 * DEGRAD
    olon = OLON * DEGRAD
    olat = OLAT * DEGRAD
    
    sn = math.tan(math.pi * 0.25 + slat2 * 0.5) / math.tan(math.pi * 0.25 + slat1 * 0.5)
    sn = math.log(math.cos(slat1) / math.cos(slat2)) / math.log(sn)
    sf = math.tan(math.pi * 0.25 + slat1 * 0.5)
    sf = math.pow(sf, sn) * math.cos(slat1) / sn
    ro = math.tan(math.pi * 0.25 + olat * 0.5)
    ro = re * sf / math.pow(ro, sn)
    
    ra = math.tan(math.pi * 0.25 + lat * DEGRAD * 0.5)
    ra = re * sf / math.pow(ra, sn)
    theta = lon * DEGRAD - olon
    if theta > math.pi:
        theta -= 2.0 * math.pi
    if theta < -math.pi:
        theta += 2.0 * math.pi
    theta *= sn
    
    nx = int(ra * math.sin(theta) + XO + 0.5)
    ny = int(ro - ra * math.cos(theta) + YO + 0.5)
    
    return {"nx": nx, "ny": ny}



@app.get("/api/weather")
async def get_weather():
    """기상청 API를 통해 현재 날씨 정보 조회 (IP 기반 위치 + 상세 정보)"""
    
    # IP 기반 위치 조회 (src/location.py의 함수 사용)
    location_info = await get_location_from_ip_async()
    location = location_info["city"]
    
    # 위경도 → 격자 좌표 변환
    grid = latlon_to_grid(location_info["lat"], location_info["lon"])
    
    # 기상청 API는 매시간 정각에 업데이트됨
    now = datetime.now()
    base_time = now.strftime("%H") + "00"
    base_date = now.strftime("%Y%m%d")
    
    # 정각 직후에는 아직 데이터가 없을 수 있으므로 1시간 전 데이터 사용
    if now.minute < 40:
        prev_hour = now.hour - 1
        if prev_hour < 0:
            prev_hour = 23
            base_date = (now - timedelta(days=1)).strftime("%Y%m%d")
        base_time = f"{prev_hour:02d}00"
    
    try:
        url = "http://apis.data.go.kr/1360000/VilageFcstInfoService_2.0/getUltraSrtNcst"
        params = {
            "serviceKey": WEATHER_API_KEY,
            "numOfRows": "10",
            "pageNo": "1",
            "dataType": "JSON",
            "base_date": base_date,
            "base_time": base_time,
            "nx": grid["nx"],
            "ny": grid["ny"],
        }
        
        async with httpx.AsyncClient(timeout=10.0) as client:
            response = await client.get(url, params=params)
            data = response.json()
        
        # 응답에서 필요한 정보 추출
        weather_data = {
            "temperature": None,
            "sky": "1",  # 기본: 맑음
            "pty": "0",  # 기본: 강수 없음
            "wind_speed": None,
            "humidity": None,
        }
        
        if "response" in data and "body" in data["response"]:
            items = data["response"]["body"].get("items", {}).get("item", [])
            for item in items:
                category = item.get("category")
                value = item.get("obsrValue")
                
                if category == "T1H":  # 기온
                    weather_data["temperature"] = float(value)
                elif category == "PTY":  # 강수형태
                    weather_data["pty"] = str(int(float(value)))
                elif category == "WSD":  # 풍속
                    weather_data["wind_speed"] = float(value)
                elif category == "REH":  # 습도
                    weather_data["humidity"] = float(value)
        
        # 하늘상태는 초단기예보에서 가져와야 함 (실황에는 없음)
        # 강수형태가 있으면 그에 맞는 아이콘 사용
        pty = weather_data["pty"]
        if pty != "0" and pty in PTY_STATUS:
            icon = PTY_STATUS[pty]["icon"]
            sky_name = PTY_STATUS[pty]["name"]
        else:
            sky_info = SKY_STATUS.get(weather_data["sky"], SKY_STATUS["1"])
            icon = sky_info["icon"]
            sky_name = sky_info["name"]
        
        return JSONResponse(content={
            "location": location,
            "region": location_info.get("region", ""),
            "temperature": weather_data["temperature"] if weather_data["temperature"] is not None else 0,
            "sky": sky_name,
            "icon": icon,
            "wind_speed": weather_data["wind_speed"] if weather_data["wind_speed"] is not None else 0,
            "humidity": weather_data["humidity"] if weather_data["humidity"] is not None else 0,
            "timestamp": now.strftime("%Y-%m-%d %H:%M"),
            "grid": grid,
        })
    
    except Exception as e:
        print(f"날씨 API 오류: {e}")
        return JSONResponse(content={
            "location": location,
            "temperature": 0,
            "sky": "맑음",
            "icon": "fa-sun",
            "wind_speed": 0,
            "error": str(e),
        })


# ============================================
# CCTV 서버 프록시 API
# ============================================
CCTV_SERVER_URL = os.getenv("CCTV_SERVER_URL")


@app.get("/api/cctv/status")
async def get_cctv_status():
    """CCTV 서버 상태 프록시 (소음 dB, 실행 상태)"""
    try:
        async with httpx.AsyncClient(timeout=5.0) as client:
            response = await client.get(f"{CCTV_SERVER_URL}/status")
            return JSONResponse(content=response.json())
    except Exception as e:
        return JSONResponse(content={"error": str(e), "db": 0, "is_running": False})


@app.post("/api/cctv/toggle_power")
async def toggle_cctv_power():
    """CCTV 서버 전원 토글 프록시"""
    try:
        async with httpx.AsyncClient(timeout=5.0) as client:
            response = await client.post(f"{CCTV_SERVER_URL}/toggle_power")
            return JSONResponse(content=response.json())
    except Exception as e:
        return JSONResponse(content={"error": str(e), "status": "error"})


@app.get("/cctv/stream")
async def cctv_stream():
    """CCTV 비디오 스트림 프록시 (MJPEG)"""
    async def generate():
        try:
            async with httpx.AsyncClient(timeout=None) as client:
                async with client.stream("GET", f"{CCTV_SERVER_URL}/video_feed") as response:
                    async for chunk in response.aiter_bytes():
                        yield chunk
        except Exception as e:
            print(f"CCTV 스트림 오류: {e}")
    
    return StreamingResponse(generate(), media_type="multipart/x-mixed-replace; boundary=frame")


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
                blank_image = np.zeros((480, 640, 3), dtype=np.uint8)
                blank_image[:] = (30, 30, 30)
                cv2.putText(
                    blank_image, "CAMERA", (230, 220),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 107, 53), 3
                )
                cv2.putText(
                    blank_image, "Waiting...", (250, 270),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (139, 142, 152), 2
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


@app.get("/camera/raw")
async def camera_raw_stream():
    """터틀봇3 카메라 원본 스트림 (YOLO 없음, 오토 라벨링용)"""

    async def generate():
        while True:
            if robot_node and robot_node.latest_raw_frame is not None:
                ret, buffer = cv2.imencode(
                    ".jpg",
                    robot_node.latest_raw_frame,
                    [cv2.IMWRITE_JPEG_QUALITY, 90],
                )
                if ret:
                    frame = buffer.tobytes()
                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                    )
            else:
                # 대기 화면
                blank_image = np.zeros((480, 640, 3), dtype=np.uint8)
                blank_image[:] = (30, 30, 30)
                cv2.putText(
                    blank_image, "RAW CAMERA", (200, 220),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.3, (255, 107, 53), 3
                )
                cv2.putText(
                    blank_image, "Waiting...", (250, 270),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (139, 142, 152), 2
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


# ============================================
# YOLO 제어 API
# ============================================

@app.get("/api/yolo/status")
async def get_yolo_status():
    """YOLO 활성화 상태 조회"""
    return JSONResponse(content={
        "enabled": yolo_detector.enabled
    })


@app.post("/api/yolo/toggle")
async def toggle_yolo():
    """YOLO ON/OFF 토글"""
    new_state = yolo_detector.toggle()
    return JSONResponse(content={
        "enabled": new_state,
        "message": f"YOLO {'활성화' if new_state else '비활성화'}됨"
    })


@app.post("/api/yolo/set")
async def set_yolo(enabled: bool):
    """YOLO 상태 직접 설정"""
    yolo_detector.enabled = enabled
    return JSONResponse(content={
        "enabled": yolo_detector.enabled
    })


# ============================================
# 감시 모드 API
# ============================================

class SurveillanceSchedule(BaseModel):
    """감시 시간대 설정 모델"""
    start_time: str  # HH:MM 형식
    end_time: str    # HH:MM 형식


@app.get("/api/surveillance/status")
async def get_surveillance_status():
    """감시 모드 상태 조회"""
    return JSONResponse(content=surveillance_system.get_status())


@app.post("/api/surveillance/start")
async def start_surveillance():
    """감시 모드 시작 (펫 모드와 상호 배제)"""
    # 펫 모드가 실행 중이면 중지
    if pet_mode_system.is_running:
        pet_mode_system.stop()
        print("🐕 펫 모드 자동 중지 (감시 모드 시작)")
    
    surveillance_system.start()
    return JSONResponse(content={
        "status": "started",
        "message": "감시 모드 시작됨"
    })


@app.post("/api/surveillance/stop")
async def stop_surveillance():
    """감시 모드 중지"""
    surveillance_system.stop()
    return JSONResponse(content={
        "status": "stopped",
        "message": "감시 모드 중지됨"
    })


@app.post("/api/surveillance/force")
async def force_surveillance(enabled: bool):
    """감시 모드 강제 활성화 (시간 무시)"""
    surveillance_system.force_enabled = enabled
    return JSONResponse(content={
        "force_enabled": surveillance_system.force_enabled,
        "message": f"강제 모드 {'활성화' if enabled else '비활성화'}됨"
    })


@app.post("/api/surveillance/schedule")
async def set_surveillance_schedule(schedule: SurveillanceSchedule):
    """감시 시간대 설정"""
    surveillance_system.set_schedule(schedule.start_time, schedule.end_time)
    return JSONResponse(content={
        "status": "ok",
        "schedule": surveillance_system.schedule
    })


# ============================================
# 펫 모드 API
# ============================================

@app.get("/api/pet/status")
async def get_pet_status():
    """펫 모드 상태 조회"""
    return JSONResponse(content=pet_mode_system.get_status())


@app.post("/api/pet/start")
async def start_pet_mode():
    """펫 모드 시작 (감시 모드와 상호 배제)"""
    # 감시 모드가 실행 중이면 중지
    if surveillance_system.is_running:
        surveillance_system.stop()
        print("🚨 감시 모드 자동 중지 (펫 모드 시작)")
    
    success = pet_mode_system.start()
    return JSONResponse(content={
        "status": "started" if success else "already_running",
        "message": "펫 모드 시작됨" if success else "이미 실행 중"
    })


@app.post("/api/pet/stop")
async def stop_pet_mode():
    """펫 모드 중지"""
    pet_mode_system.stop()
    return JSONResponse(content={
        "status": "stopped",
        "message": "펫 모드 중지됨"
    })


if __name__ == "__main__":
    uvicorn.run("main:app", host=HOST, port=PORT, reload=True)