"""
Pet Mode System
- Follow owner (YOLO class_id=1) while maintaining target distance
- Use Lidar for distance measurement
- Use Nav2 for navigation
- Rotate in place when owner is lost
"""

import threading
import time
import math
from typing import Callable, Optional, Tuple

from detect.yolo_detector import yolo_detector


class PetModeSystem:
    """
    Pet Mode System - Follow owner like a pet
    """
    
    # Target distance to maintain from owner (meters)
    TARGET_DISTANCE = 0.4  # 40cm
    
    # Distance tolerance (meters)
    DISTANCE_TOLERANCE = 0.1  # 10cm tolerance
    
    # Camera field of view (degrees) - TurtleBot3 camera approx 60-70 deg
    CAMERA_FOV = 62.0
    
    # Search rotation speed (rad/s)
    SEARCH_ANGULAR_SPEED = 0.3
    
    # Owner class ID (from classes.yaml: 1 = owner)
    OWNER_CLASS_ID = 1
    
    def __init__(self):
        self._running = False
        self._lock = threading.Lock()
        self._tracking_thread: Optional[threading.Thread] = None
        
        # State
        self._state = "idle"  # idle, searching, following
        self._owner_detected = False
        self._owner_position = None  # (angle, distance)
        
        # Callbacks (set from main.py)
        self._get_camera_frame: Optional[Callable] = None
        self._get_lidar_scan: Optional[Callable] = None
        self._send_nav_goal: Optional[Callable] = None
        self._send_cmd_vel: Optional[Callable] = None
        self._get_robot_pose: Optional[Callable] = None
        
        # Last known owner position (for re-acquisition)
        self._last_owner_angle = 0.0
        
    @property
    def is_running(self) -> bool:
        """Pet mode running status"""
        with self._lock:
            return self._running
    
    @property
    def state(self) -> str:
        """Current state: idle, searching, following"""
        with self._lock:
            return self._state
    
    def set_callbacks(self,
                      get_camera_frame: Callable = None,
                      get_lidar_scan: Callable = None,
                      send_nav_goal: Callable = None,
                      send_cmd_vel: Callable = None,
                      get_robot_pose: Callable = None):
        """
        Set callback functions from main.py
        
        Args:
            get_camera_frame: Get current camera frame
            get_lidar_scan: Get current lidar scan data
            send_nav_goal: Send Nav2 goal (x, y) -> bool
            send_cmd_vel: Send velocity command (linear, angular)
            get_robot_pose: Get current robot pose (x, y, theta)
        """
        self._get_camera_frame = get_camera_frame
        self._get_lidar_scan = get_lidar_scan
        self._send_nav_goal = send_nav_goal
        self._send_cmd_vel = send_cmd_vel
        self._get_robot_pose = get_robot_pose
    
    def start(self) -> bool:
        """
        Start pet mode
        
        Returns:
            bool: True if started successfully
        """
        with self._lock:
            if self._running:
                return False
            self._running = True
            self._state = "searching"
        
        # Enable YOLO detection
        yolo_detector.enabled = True
        
        # Start tracking thread
        self._tracking_thread = threading.Thread(target=self._tracking_loop, daemon=True)
        self._tracking_thread.start()
        
        print("🐕 Pet Mode started!")
        return True
    
    def stop(self):
        """Stop pet mode"""
        with self._lock:
            self._running = False
            self._state = "idle"
        
        # Stop robot movement
        if self._send_cmd_vel:
            self._send_cmd_vel(0.0, 0.0)
        
        if self._tracking_thread:
            self._tracking_thread.join(timeout=2.0)
        
        print("⏹️ Pet Mode stopped")
    
    def _tracking_loop(self):
        """Main tracking loop (runs in separate thread)"""
        while self.is_running:
            try:
                # 1. Detect owner in camera
                owner_bbox = self._detect_owner()
                
                if owner_bbox:
                    # Owner detected
                    self._owner_detected = True
                    with self._lock:
                        self._state = "following"
                    
                    # 2. Get distance from lidar
                    angle = self._bbox_to_angle(owner_bbox)
                    distance = self._get_distance_at_angle(angle)
                    
                    self._last_owner_angle = angle
                    self._owner_position = (angle, distance)
                    
                    # 3. Follow owner
                    self._follow_owner(angle, distance)
                    
                else:
                    # Owner lost
                    self._owner_detected = False
                    with self._lock:
                        self._state = "searching"
                    
                    # 4. Search by rotating
                    self._search_owner()
                
                time.sleep(0.1)  # 10Hz control loop
                
            except Exception as e:
                print(f"⚠️ Pet Mode tracking error: {e}")
                time.sleep(0.5)
    
    def _detect_owner(self) -> Optional[dict]:
        """
        Detect owner in camera frame
        
        Returns:
            Owner detection dict or None
        """
        if not self._get_camera_frame:
            return None
        
        frame = self._get_camera_frame()
        if frame is None:
            return None
        
        # Get detections from YOLO
        detections = yolo_detector.detect_persons(frame)
        
        # Find owner (class_id = 1)
        for det in detections:
            if det.get("class_id") == self.OWNER_CLASS_ID:
                return det
        
        return None
    
    def _bbox_to_angle(self, detection: dict) -> float:
        """
        Convert bounding box center to angle (radians)
        
        Assumes camera image width is 640 pixels
        Camera center is at x=320
        
        Returns:
            Angle in radians (positive = left, negative = right)
        """
        bbox = detection["bbox"]
        x1, y1, x2, y2 = bbox
        center_x = (x1 + x2) / 2
        
        # Image width (assume 640)
        image_width = 640
        
        # Normalize to -0.5 to 0.5
        normalized_x = (center_x - image_width / 2) / image_width
        
        # Convert to angle (negative because camera x increases to right)
        fov_rad = math.radians(self.CAMERA_FOV)
        angle = -normalized_x * fov_rad
        
        return angle
    
    def _get_distance_at_angle(self, angle: float) -> float:
        """
        Get distance from lidar at specified angle
        
        Args:
            angle: Angle in radians (0 = front, positive = left, negative = right)
            
        Returns:
            Distance in meters, or inf if no reading
        """
        if not self._get_lidar_scan:
            return float('inf')
        
        scan = self._get_lidar_scan()
        if scan is None:
            return float('inf')
        
        # LaserScan message fields
        ranges = scan.ranges
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        
        # TurtleBot3 lidar: angle_min is typically 0 (front) going counter-clockwise
        # Camera angle: 0 = front, positive = left, negative = right
        # Convert camera angle to lidar index
        # If angle_min is negative (e.g., -π), adjust accordingly
        
        lidar_angle = angle
        
        # Handle angle wrapping (lidar might be 0 to 2π or -π to π)
        if angle_min >= 0:
            # Lidar is 0 to 2π format
            if lidar_angle < 0:
                lidar_angle = lidar_angle + 2 * math.pi
        
        # Calculate index from angle
        index = int((lidar_angle - angle_min) / angle_increment)
        
        # Wrap index if needed
        num_readings = len(ranges)
        index = index % num_readings
        
        if 0 <= index < num_readings:
            distance = ranges[index]
            # Check for valid reading (filter inf and nan)
            if distance > 0.01 and distance < 10.0 and not math.isinf(distance) and not math.isnan(distance):
                return distance
        
        # If no valid reading, try nearby indices
        for offset in range(1, 10):
            for idx in [index + offset, index - offset]:
                idx = idx % num_readings
                if 0 <= idx < num_readings:
                    d = ranges[idx]
                    if d > 0.01 and d < 10.0 and not math.isinf(d) and not math.isnan(d):
                        return d
        
        return float('inf')
    
    def _follow_owner(self, angle: float, distance: float):
        """
        Follow owner - first rotate to center, then move forward
        
        Args:
            angle: Owner angle from robot (radians)
            distance: Owner distance from robot (meters)
        """
        if not self._send_cmd_vel:
            return
        
        # Angle threshold for "centered" (about 5 degrees)
        CENTER_THRESHOLD = 0.1  # radians (~5.7 degrees)
        
        # Calculate distance error
        distance_error = distance - self.TARGET_DISTANCE
        
        # Check if owner is centered in camera
        if abs(angle) > CENTER_THRESHOLD:
            # Step 1: ROTATE ONLY to center owner
            # No forward movement while rotating
            angular_vel = angle * 0.8  # P-controller gain
            angular_vel = max(-0.4, min(0.4, angular_vel))  # Clamp
            linear_vel = 0.0
            
            print(f"🔄 Centering owner: angle={math.degrees(angle):.1f}°, rotating...")
        else:
            # Step 2: MOVE FORWARD (owner is centered)
            # Minimal angular correction while moving
            if abs(distance_error) > self.DISTANCE_TOLERANCE:
                linear_vel = min(0.15, max(-0.08, distance_error * 0.3))
            else:
                linear_vel = 0.0
            
            # Very small angular correction only
            angular_vel = angle * 0.3
            angular_vel = max(-0.15, min(0.15, angular_vel))
            
            print(f"🐕 Moving to owner: dist={distance:.2f}m (err={distance_error:+.2f}m), vel=({linear_vel:.2f}, {angular_vel:.2f})")
        
        self._send_cmd_vel(linear_vel, angular_vel)
    
    def _search_owner(self):
        """Rotate in place to search for owner"""
        if not self._send_cmd_vel:
            return
        
        # Determine rotation direction based on last known position
        if self._last_owner_angle >= 0:
            angular = self.SEARCH_ANGULAR_SPEED  # Rotate left
        else:
            angular = -self.SEARCH_ANGULAR_SPEED  # Rotate right
        
        self._send_cmd_vel(0.0, angular)
        print(f"🔍 Searching for owner... rotating {'left' if angular > 0 else 'right'}")
    
    def get_status(self) -> dict:
        """Get current pet mode status"""
        pos = self._owner_position
        return {
            "is_running": self.is_running,
            "state": self.state,
            "owner_detected": self._owner_detected,
            "owner_position": list(pos) if pos else None,
            "target_distance": self.TARGET_DISTANCE,
            "yolo_enabled": yolo_detector.enabled
        }


# Global pet mode instance
pet_mode_system = PetModeSystem()
