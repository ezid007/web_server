/**
 * Nav2 맵 뷰어 및 Goal 설정 모듈
 * rosbridge를 통해 ROS2와 통신하여 맵 시각화 및 네비게이션 Goal 전송
 */

class Nav2MapViewer {
    constructor(canvasId, options = {}) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');
        
        // 설정
        this.rosbridgeUrl = options.rosbridgeUrl || 'ws://localhost:9090';
        this.mapTopic = options.mapTopic || '/map';
        this.poseTopic = options.poseTopic || '/amcl_pose';
        this.goalTopic = options.goalTopic || '/goal_pose';
        
        // 맵 데이터
        this.mapData = null;
        this.mapInfo = null;
        this.robotPose = null;
        this.goalPosition = null; // Goal 위치 저장
        
        // 뷰 상태
        this.scale = 1.0;
        this.offsetX = 0;
        this.offsetY = 0;
        this.isDragging = false;
        this.lastMouseX = 0;
        this.lastMouseY = 0;
        
        // ROS 연결
        this.ros = null;
        this.connected = false;
        
        // 첫 로드 플래그
        this.firstMapLoad = true;
        
        // 이벤트 바인딩
        this.bindEvents();
    }
    
    /**
     * rosbridge 연결
     */
    connect() {
        this.ros = new ROSLIB.Ros({ url: this.rosbridgeUrl });
        
        this.ros.on('connection', () => {
            console.log('✅ rosbridge 연결됨');
            this.connected = true;
            this.subscribeTopics();
            this.updateStatus('연결됨');
        });
        
        this.ros.on('error', (error) => {
            console.error('❌ rosbridge 오류:', error);
            this.updateStatus('오류');
        });
        
        this.ros.on('close', () => {
            console.log('🔌 rosbridge 연결 해제됨');
            this.connected = false;
            this.updateStatus('연결 해제');
            // 5초 후 재연결 시도
            setTimeout(() => this.connect(), 5000);
        });
    }
    
    /**
     * ROS 토픽 구독
     */
    subscribeTopics() {
        // 맵 토픽 구독
        const mapListener = new ROSLIB.Topic({
            ros: this.ros,
            name: this.mapTopic,
            messageType: 'nav_msgs/msg/OccupancyGrid'
        });
        
        mapListener.subscribe((message) => {
            this.mapInfo = message.info;
            this.mapData = message.data;
            
            // 첫 로드 시 자동 스케일링 및 중앙 정렬
            if (this.firstMapLoad) {
                this.autoFitMap();
                this.firstMapLoad = false;
            }
            
            this.renderMap();
        });
        
        // 로봇 위치 토픽 구독
        const poseListener = new ROSLIB.Topic({
            ros: this.ros,
            name: this.poseTopic,
            messageType: 'geometry_msgs/msg/PoseWithCovarianceStamped'
        });
        
        poseListener.subscribe((message) => {
            this.robotPose = message.pose.pose;
            
            // Goal에 도달했는지 확인 (임계값: 0.3m)
            if (this.goalPosition && this.robotPose) {
                const dx = this.robotPose.position.x - this.goalPosition.x;
                const dy = this.robotPose.position.y - this.goalPosition.y;
                const distance = Math.sqrt(dx * dx + dy * dy);
                if (distance < 0.3) {
                    console.log('✅ Goal 도달!');
                    this.goalPosition = null; // Goal 마커 제거
                }
            }
            
            this.renderMap(); // 전체 다시 그리기
        });
    }
    
    /**
     * 맵 렌더링
     */
    renderMap() {
        if (!this.mapData || !this.mapInfo) return;
        
        const width = this.mapInfo.width;
        const height = this.mapInfo.height;
        
        // ImageData 생성
        const imageData = this.ctx.createImageData(width, height);
        
        for (let i = 0; i < this.mapData.length; i++) {
            const value = this.mapData[i];
            let color;
            
            if (value === -1) {
                // 알 수 없음 - 회색
                color = [128, 128, 128, 255];
            } else if (value === 0) {
                // 빈 공간 - 흰색
                color = [255, 255, 255, 255];
            } else {
                // 장애물 - 검정색 (값이 높을수록 진함)
                const shade = Math.max(0, 255 - value * 2.55);
                color = [shade, shade, shade, 255];
            }
            
            const idx = i * 4;
            imageData.data[idx] = color[0];
            imageData.data[idx + 1] = color[1];
            imageData.data[idx + 2] = color[2];
            imageData.data[idx + 3] = color[3];
        }
        
        // 임시 캔버스에 맵 그리기
        const tempCanvas = document.createElement('canvas');
        tempCanvas.width = width;
        tempCanvas.height = height;
        const tempCtx = tempCanvas.getContext('2d');
        tempCtx.putImageData(imageData, 0, 0);
        
        // 메인 캔버스에 스케일 적용하여 그리기
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        this.ctx.fillStyle = '#1e1e1e';
        this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
        
        this.ctx.save();
        this.ctx.translate(this.offsetX, this.offsetY);
        this.ctx.scale(this.scale, this.scale);
        
        // 맵 이미지를 뒤집어서 그리기 (ROS 좌표계 → 화면 좌표계)
        this.ctx.translate(0, height);
        this.ctx.scale(1, -1);
        this.ctx.drawImage(tempCanvas, 0, 0);
        
        this.ctx.restore();
        
        // 로봇 위치 다시 그리기
        this.renderRobot();
        
        // Goal 마커 그리기
        this.renderGoalMarker();
    }
    
    /**
     * 로봇 위치 렌더링
     */
    renderRobot() {
        if (!this.robotPose || !this.mapInfo) return;
        
        const resolution = this.mapInfo.resolution;
        const originX = this.mapInfo.origin.position.x;
        const originY = this.mapInfo.origin.position.y;
        
        // 맵 좌표 → 픽셀 좌표 변환
        const robotX = (this.robotPose.position.x - originX) / resolution;
        const robotY = (this.robotPose.position.y - originY) / resolution;
        
        // 화면 좌표로 변환 (Y축 뒤집기)
        const screenX = this.offsetX + robotX * this.scale;
        const screenY = this.offsetY + (this.mapInfo.height - robotY) * this.scale;
        
        // 방향 (quaternion → yaw)
        const q = this.robotPose.orientation;
        const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        
        // 로봇 화살표 그리기
        this.ctx.save();
        this.ctx.translate(screenX, screenY);
        this.ctx.rotate(-yaw); // 화면 좌표계에서 회전
        
        // 화살표 모양
        this.ctx.beginPath();
        this.ctx.fillStyle = '#00ff88';
        this.ctx.moveTo(15, 0);
        this.ctx.lineTo(-10, -8);
        this.ctx.lineTo(-5, 0);
        this.ctx.lineTo(-10, 8);
        this.ctx.closePath();
        this.ctx.fill();
        
        // 테두리
        this.ctx.strokeStyle = '#003322';
        this.ctx.lineWidth = 2;
        this.ctx.stroke();
        
        this.ctx.restore();
    }
    
    /**
     * Goal 전송
     */
    sendGoal(screenX, screenY) {
        if (!this.connected || !this.mapInfo) {
            console.warn('맵이 로드되지 않았거나 연결되지 않음');
            return;
        }
        
        const resolution = this.mapInfo.resolution;
        const originX = this.mapInfo.origin.position.x;
        const originY = this.mapInfo.origin.position.y;
        
        // 화면 좌표 → 맵 좌표 변환
        const mapX = (screenX - this.offsetX) / this.scale;
        const mapY = (screenY - this.offsetY) / this.scale;
        
        // 픽셀 좌표 → 미터 좌표 (Y축 뒤집기)
        const worldX = mapX * resolution + originX;
        const worldY = (this.mapInfo.height - mapY) * resolution + originY;
        
        // Goal 메시지 생성
        const goalTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: this.goalTopic,
            messageType: 'geometry_msgs/msg/PoseStamped'
        });
        
        const goal = new ROSLIB.Message({
            header: {
                stamp: { sec: 0, nanosec: 0 },
                frame_id: 'map'
            },
            pose: {
                position: { x: worldX, y: worldY, z: 0.0 },
                orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
            }
        });
        
        goalTopic.publish(goal);
        console.log(`🎯 Goal 전송: (${worldX.toFixed(2)}, ${worldY.toFixed(2)})`);
        
        // Goal 위치 저장 (렌더링에 사용)
        this.goalPosition = { x: worldX, y: worldY, screenX: screenX, screenY: screenY };
        
        // 즉시 화면에 표시
        this.renderMap();
    }
    
    /**
     * Nav2 Goal 취소 (긴급 정지용)
     */
    cancelGoal() {
        if (!this.connected) {
            console.warn('rosbridge 연결 안됨');
            return;
        }
        
        // Nav2 navigate_to_pose 액션 취소
        const cancelClient = new ROSLIB.Service({
            ros: this.ros,
            name: '/navigate_to_pose/_action/cancel_goal',
            serviceType: 'action_msgs/srv/CancelGoal'
        });
        
        const request = new ROSLIB.ServiceRequest({
            goal_info: {
                goal_id: { uuid: Array(16).fill(0) },  // 모든 Goal 취소
                stamp: { sec: 0, nanosec: 0 }
            }
        });
        
        cancelClient.callService(request, (result) => {
            console.log('🛑 Nav2 Goal 취소됨:', result);
        }, (error) => {
            console.error('Nav2 Goal 취소 실패:', error);
        });
        
        // Goal 마커도 제거
        this.goalPosition = null;
        this.renderMap();
        
        console.log('🛑 Nav2 네비게이션 취소 요청됨');
    }
    
    /**
     * Goal 마커 렌더링
     */
    renderGoalMarker() {
        if (!this.goalPosition || !this.mapInfo) return;
        
        const resolution = this.mapInfo.resolution;
        const originX = this.mapInfo.origin.position.x;
        const originY = this.mapInfo.origin.position.y;
        
        // 월드 좌표 → 화면 좌표
        const goalMapX = (this.goalPosition.x - originX) / resolution;
        const goalMapY = (this.goalPosition.y - originY) / resolution;
        const x = this.offsetX + goalMapX * this.scale;
        const y = this.offsetY + (this.mapInfo.height - goalMapY) * this.scale;
        
        // Goal 마커 그리기
        this.ctx.beginPath();
        this.ctx.arc(x, y, 12, 0, Math.PI * 2);
        this.ctx.fillStyle = 'rgba(255, 107, 53, 0.8)';
        this.ctx.fill();
        this.ctx.strokeStyle = '#ff6b35';
        this.ctx.lineWidth = 3;
        this.ctx.stroke();
        
        // X 표시
        this.ctx.beginPath();
        this.ctx.strokeStyle = '#ffffff';
        this.ctx.lineWidth = 2;
        this.ctx.moveTo(x - 6, y - 6);
        this.ctx.lineTo(x + 6, y + 6);
        this.ctx.moveTo(x + 6, y - 6);
        this.ctx.lineTo(x - 6, y + 6);
        this.ctx.stroke();
    }
    
    /**
     * 이벤트 바인딩
     */
    bindEvents() {
        // 클릭/터치로 Goal 설정
        this.canvas.addEventListener('click', (e) => {
            const rect = this.canvas.getBoundingClientRect();
            // CSS 크기와 캔버스 속성 크기 비율 계산
            const scaleX = this.canvas.width / rect.width;
            const scaleY = this.canvas.height / rect.height;
            const x = (e.clientX - rect.left) * scaleX;
            const y = (e.clientY - rect.top) * scaleY;
            this.sendGoal(x, y);
        });
        
        // 터치 이벤트
        this.canvas.addEventListener('touchend', (e) => {
            e.preventDefault();
            if (e.changedTouches.length > 0) {
                const touch = e.changedTouches[0];
                const rect = this.canvas.getBoundingClientRect();
                // CSS 크기와 캔버스 속성 크기 비율 계산
                const scaleX = this.canvas.width / rect.width;
                const scaleY = this.canvas.height / rect.height;
                const x = (touch.clientX - rect.left) * scaleX;
                const y = (touch.clientY - rect.top) * scaleY;
                this.sendGoal(x, y);
            }
        }, { passive: false });
        
        // 마우스 휠로 줌
        this.canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            const delta = e.deltaY > 0 ? 0.9 : 1.1;
            this.scale *= delta;
            this.scale = Math.max(0.5, Math.min(5, this.scale));
            this.renderMap();
        }, { passive: false });
        
        // 드래그로 이동
        this.canvas.addEventListener('mousedown', (e) => {
            if (e.button === 2 || e.ctrlKey) { // 우클릭 또는 Ctrl+클릭
                this.isDragging = true;
                this.lastMouseX = e.clientX;
                this.lastMouseY = e.clientY;
            }
        });
        
        this.canvas.addEventListener('mousemove', (e) => {
            if (this.isDragging) {
                this.offsetX += e.clientX - this.lastMouseX;
                this.offsetY += e.clientY - this.lastMouseY;
                this.lastMouseX = e.clientX;
                this.lastMouseY = e.clientY;
                this.renderMap();
            }
        });
        
        this.canvas.addEventListener('mouseup', () => {
            this.isDragging = false;
        });
        
        this.canvas.addEventListener('mouseleave', () => {
            this.isDragging = false;
        });
        
        // 우클릭 메뉴 방지
        this.canvas.addEventListener('contextmenu', (e) => {
            e.preventDefault();
        });
    }
    
    /**
     * 상태 업데이트 (UI)
     */
    updateStatus(status) {
        const statusEl = document.getElementById('nav2-status');
        if (statusEl) {
            statusEl.textContent = status;
            statusEl.className = 'nav2-status ' + (this.connected ? 'connected' : 'disconnected');
        }
    }
    
    /**
     * 맵을 화면에 맞게 자동 스케일링
     */
    autoFitMap() {
        if (!this.mapInfo) return;
        
        const canvasWidth = this.canvas.width;
        const canvasHeight = this.canvas.height;
        const mapWidth = this.mapInfo.width;
        const mapHeight = this.mapInfo.height;
        
        // 여백을 두고 화면에 맞게 스케일 계산
        const padding = 40;
        const scaleX = (canvasWidth - padding * 2) / mapWidth;
        const scaleY = (canvasHeight - padding * 2) / mapHeight;
        this.scale = Math.min(scaleX, scaleY);
        
        // 중앙 정렬
        this.offsetX = (canvasWidth - mapWidth * this.scale) / 2;
        this.offsetY = (canvasHeight - mapHeight * this.scale) / 2;
        
        console.log(`🗺️ 맵 자동 스케일: ${this.scale.toFixed(2)}x, 오프셋: (${this.offsetX.toFixed(0)}, ${this.offsetY.toFixed(0)})`);
    }
    
    /**
     * 초기 맵 위치 설정 (버튼 클릭 시)
     */
    centerMap() {
        this.autoFitMap();
        this.renderMap();
    }
}

// 전역 인스턴스
let nav2MapViewer = null;

/**
 * Nav2 맵 뷰어 초기화
 */
function initNav2MapViewer(rosbridgeUrl) {
    nav2MapViewer = new Nav2MapViewer('nav2Canvas', {
        rosbridgeUrl: rosbridgeUrl || 'ws://localhost:9090'
    });
    nav2MapViewer.connect();
}
