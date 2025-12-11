"""
감시 모드 시스템
- 지정된 좌표 순환 순찰
- CCTV에서 사람 감지 시 문 좌표로 이동
- 터틀봇 카메라로 확인 후 사진 촬영
"""

import cv2
import threading
import time
from pathlib import Path
from datetime import datetime, time as dt_time
from typing import Callable, Optional, List, Tuple

from detect.yolo_detector import yolo_detector


class SurveillanceSystem:
    """
    감시 모드 시스템
    """
    
    # 순환 좌표 (사용자 지정)
    PATROL_POINTS: List[Tuple[float, float]] = [
        (2.1, -0.644),
        (1.48, -6.93),
        (-1.0, -6.92)
    ]
    
    # 문 좌표 (CCTV 감지 시 이동)
    DOOR_POINT: Tuple[float, float] = (-1.96, -6.9)
    
    # 기본 감시 시간대
    DEFAULT_START_TIME = dt_time(23, 0)  # PM 11:00
    DEFAULT_END_TIME = dt_time(7, 0)     # AM 7:00
    
    # 사진 저장 경로
    PHOTO_DIR = Path(__file__).parent / "photo"
    
    def __init__(self):
        self._running = False
        self._force_enabled = False  # 강제 활성화 (시간 무시)
        self._patrol_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        
        # 스케줄 설정
        self._start_time = self.DEFAULT_START_TIME
        self._end_time = self.DEFAULT_END_TIME
        
        # 현재 순찰 상태
        self._current_point_index = 0
        self._is_patrolling = False
        self._is_responding_to_cctv = False
        
        # 콜백 함수들 (main.py에서 설정)
        self._send_nav_goal: Optional[Callable] = None
        self._get_robot_frame: Optional[Callable] = None
        self._get_cctv_frame: Optional[Callable] = None
        
        # 사진 폴더 생성
        self.PHOTO_DIR.mkdir(parents=True, exist_ok=True)
    
    @property
    def is_running(self) -> bool:
        """감시 모드 실행 중 여부"""
        with self._lock:
            return self._running
    
    @property
    def force_enabled(self) -> bool:
        """강제 활성화 상태"""
        with self._lock:
            return self._force_enabled
    
    @force_enabled.setter
    def force_enabled(self, value: bool):
        with self._lock:
            self._force_enabled = value
        if value:
            self.start()
    
    @property
    def schedule(self) -> dict:
        """현재 스케줄 반환"""
        return {
            "start_time": self._start_time.strftime("%H:%M"),
            "end_time": self._end_time.strftime("%H:%M")
        }
    
    def set_schedule(self, start_time: str, end_time: str):
        """
        감시 시간대 설정
        
        Args:
            start_time: 시작 시간 (HH:MM 형식)
            end_time: 종료 시간 (HH:MM 형식)
        """
        try:
            h, m = map(int, start_time.split(":"))
            self._start_time = dt_time(h, m)
            
            h, m = map(int, end_time.split(":"))
            self._end_time = dt_time(h, m)
            
            print(f"📅 감시 시간대 설정: {start_time} ~ {end_time}")
        except Exception as e:
            print(f"❌ 시간대 설정 오류: {e}")
    
    def is_within_schedule(self) -> bool:
        """현재 시간이 감시 시간대 내인지 확인"""
        if self._force_enabled:
            return True
            
        now = datetime.now().time()
        
        # 야간 시간대 (예: 23:00 ~ 07:00)
        if self._start_time > self._end_time:
            return now >= self._start_time or now <= self._end_time
        else:
            return self._start_time <= now <= self._end_time
    
    def set_callbacks(self, 
                      send_nav_goal: Callable = None,
                      get_robot_frame: Callable = None,
                      get_cctv_frame: Callable = None):
        """
        콜백 함수 설정
        
        Args:
            send_nav_goal: Nav2 Goal 전송 함수 (x, y) -> bool
            get_robot_frame: 터틀봇 카메라 프레임 가져오기 () -> np.ndarray
            get_cctv_frame: CCTV 프레임 가져오기 () -> np.ndarray
        """
        self._send_nav_goal = send_nav_goal
        self._get_robot_frame = get_robot_frame
        self._get_cctv_frame = get_cctv_frame
    
    def start(self):
        """감시 모드 시작"""
        with self._lock:
            if self._running:
                return
            self._running = True
        
        # YOLO 활성화
        yolo_detector.enabled = True
        
        # 순찰 스레드 시작
        self._patrol_thread = threading.Thread(target=self._patrol_loop, daemon=True)
        self._patrol_thread.start()
        
        print("🚨 감시 모드 시작!")
    
    def stop(self):
        """감시 모드 중지"""
        with self._lock:
            self._running = False
            self._force_enabled = False
        
        if self._patrol_thread:
            self._patrol_thread.join(timeout=2.0)
        
        print("⏹️ 감시 모드 중지")
    
    def _patrol_loop(self):
        """순찰 루프 (별도 스레드)"""
        while self.is_running:
            # 시간대 확인
            if not self.is_within_schedule():
                time.sleep(60)  # 1분마다 확인
                continue
            
            # 현재 순찰 포인트로 이동
            if not self._is_responding_to_cctv:
                self._move_to_next_patrol_point()
            
            # CCTV 감시 (1초마다)
            time.sleep(1)
            self._check_cctv()
    
    def _move_to_next_patrol_point(self):
        """다음 순찰 포인트로 이동"""
        if not self._send_nav_goal:
            return
            
        point = self.PATROL_POINTS[self._current_point_index]
        x, y = point
        
        print(f"🚶 순찰 포인트 {self._current_point_index + 1} 이동: ({x}, {y})")
        
        self._is_patrolling = True
        success = self._send_nav_goal(x, y)
        
        if success:
            # 다음 포인트로 인덱스 이동
            self._current_point_index = (self._current_point_index + 1) % len(self.PATROL_POINTS)
        
        # 도착 대기 (간단한 시뮬레이션 - 실제로는 Nav2 피드백 사용)
        time.sleep(10)
        self._is_patrolling = False
    
    def _check_cctv(self):
        """CCTV에서 사람 감지 확인"""
        if not self._get_cctv_frame or self._is_responding_to_cctv:
            return
            
        frame = self._get_cctv_frame()
        if frame is None:
            return
            
        # YOLO로 사람 감지
        persons = yolo_detector.detect_persons(frame)
        
        if len(persons) > 0:
            print(f"🚨 CCTV에서 사람 감지! ({len(persons)}명)")
            self._respond_to_cctv_detection()
    
    def _respond_to_cctv_detection(self):
        """CCTV 감지 시 대응"""
        self._is_responding_to_cctv = True
        
        # 문 좌표로 이동
        if self._send_nav_goal:
            x, y = self.DOOR_POINT
            print(f"🚪 문 좌표로 이동: ({x}, {y})")
            self._send_nav_goal(x, y)
            
            # 도착 대기
            time.sleep(15)
        
        # 터틀봇 카메라로 확인
        if self._get_robot_frame:
            frame = self._get_robot_frame()
            if frame is not None:
                persons = yolo_detector.detect_persons(frame)
                
                if len(persons) > 0:
                    print(f"📸 터틀봇 카메라에서 사람 확인! 사진 촬영")
                    self._capture_photo(frame, persons)
                else:
                    print("❌ 터틀봇 카메라에서 사람 미확인")
        
        self._is_responding_to_cctv = False
    
    def _capture_photo(self, frame, detections: list):
        """사진 촬영 및 저장"""
        try:
            # 바운딩 박스 그리기
            display = yolo_detector.draw_detections(frame, detections)
            
            # 타임스탬프 추가
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            cv2.putText(display, timestamp, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 파일 저장
            filename = f"detected_{timestamp}.jpg"
            filepath = self.PHOTO_DIR / filename
            cv2.imwrite(str(filepath), display)
            
            print(f"💾 사진 저장: {filepath}")
            
        except Exception as e:
            print(f"❌ 사진 저장 오류: {e}")
    
    def get_status(self) -> dict:
        """현재 상태 반환"""
        return {
            "is_running": self.is_running,
            "force_enabled": self._force_enabled,
            "is_patrolling": self._is_patrolling,
            "is_responding_to_cctv": self._is_responding_to_cctv,
            "current_patrol_index": self._current_point_index,
            "within_schedule": self.is_within_schedule(),
            "schedule": self.schedule,
            "yolo_enabled": yolo_detector.enabled
        }


# 전역 감시 시스템 인스턴스
surveillance_system = SurveillanceSystem()
