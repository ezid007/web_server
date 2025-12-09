# Web Server - 로봇 대시보드 프로젝트

TurtleBot3 제어 및 모니터링을 위한 웹 기반 대시보드 서버입니다.

## 📁 프로젝트 구조

```
web_server/
├── docs/                          # 📚 문서 모음
│   ├── 01_DEPLOYMENT_GUIDE.md     # 서버 배포 및 실행 가이드
│   └── 02_ROS2_NETWORK_CONFIGURATION.md  # ROS2 네트워크 설정 가이드
├── frontend/                      # ⚛️ React SPA (Vite)
│   ├── src/
│   │   ├── components/            # React 컴포넌트
│   │   └── utils/                 # 유틸리티 함수
│   └── dist/                      # 빌드 출력
├── src/robotis/                   # 🤖 ROS2 패키지
│   ├── launch/                    # Launch 파일
│   └── nodes/                     # ROS2 노드
├── static/                        # 🎨 정적 파일 (이미지, 비디오, CSS, JS 등)
├── templates/                     # 📄 HTML 템플릿
├── main.py                        # 🚀 FastAPI 메인 서버
├── pyproject.toml                 # 📦 Python 의존성 (Poetry)
├── cyclonedds.xml                 # (삭제됨) → /home/tuf/ros2_config/cyclonedds.xml 참고
└── README.md                      # 📖 이 파일
```

## 🚀 빠른 시작

### 1. 의존성 설치

```bash
# Python 패키지
cd /home/tuf/web_server
poetry install

# Node.js 패키지 (프론트엔드)
cd frontend
npm install
```

### 2. 프론트엔드 빌드

```bash
cd /home/tuf/web_server/frontend
npm run build
```

### 3. 서버 실행

```bash
cd /home/tuf/web_server
poetry run python main.py
```

서버가 `http://0.0.0.0:8000`에서 실행됩니다.

## 📚 문서

자세한 내용은 `docs/` 폴더의 문서를 참고하세요:

-   **[배포 가이드](docs/01_DEPLOYMENT_GUIDE.md)** - 서버 설치, 배포, systemd 설정
-   **[ROS2 네트워크 설정](docs/02_ROS2_NETWORK_CONFIGURATION.md)** - DDS, CycloneDDS 무선랜 설정

## 🛠️ 기술 스택

### 백엔드

-   **FastAPI** - Python 웹 프레임워크
-   **Poetry** - 의존성 관리
-   **ROS2 Humble** - 로봇 미들웨어

### 프론트엔드

-   **React 18** - UI 라이브러리
-   **Vite 7** - 빌드 도구
-   **React Router** - 페이지 라우팅

### 로봇 통신

-   **ROS2** - 로봇 운영체제
-   **CycloneDDS** - 데이터 배포 서비스
-   **Rosbridge** - ROS-웹 통신 브릿지

## 🌐 주요 기능

-   📱 반응형 웹 대시보드
-   🗺️ Navigation2 기반 자율주행
-   📊 실시간 센서 모니터링 (배터리, IMU, 카메라 등)
-   🎮 웹 기반 로봇 제어
-   🔌 Rosbridge를 통한 웹소켓 통신

## 💻 네트워크 구조

```
┌──────────────────┐
│   웹 브라우저     │ (http://localhost:8000)
└────────┬─────────┘
         │ HTTP
         ▼
┌──────────────────┐         ┌─────────────────┐
│  FastAPI 서버    │◄──────►│  TurtleBot3     │
│  (유선랜)        │  ROS2  │  (무선랜)       │
└──────────────────┘ DDS    └─────────────────┘
```

## ⚙️ 설정 파일

| 파일                                               | 용도                              |
| -------------------------------------------------- | --------------------------------- |
| `pyproject.toml`                                   | Python 의존성 (Poetry)            |
| `frontend/package.json`                            | Node.js 의존성                    |
| `frontend/vite.config.js`                          | Vite 빌드 설정                    |
| `/home/tuf/ros2_config/cyclonedds.xml`             | ROS2 네트워크 인터페이스 (무선랜) |
| `src/robotis/launch/robotis_navigation2.launch.py` | ROS2 Navigation2 + Rosbridge 실행 |

## 📝 환경 변수

ROS2 통신을 위해 `~/.bashrc`에 추가:

```bash
# ROS2 설정
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/tuf/ros2_config/cyclonedds.xml
export ROS_DOMAIN_ID=7
```

## 🔧 문제 해결

### ROS2 토픽이 보이지 않을 때

1. 환경변수 확인:

    ```bash
    echo $RMW_IMPLEMENTATION
    echo $CYCLONEDDS_URI
    ```

2. 네트워크 인터페이스 확인:

    ```bash
    ip addr show wlp3s0
    ```

3. 로봇 연결 확인:
    ```bash
    ros2 topic list
    ```

더 자세한 내용은 [ROS2 네트워크 설정](docs/02_ROS2_NETWORK_CONFIGURATION.md) 문서를 참고하세요.

## 📄 라이선스

자유롭게 수정 및 배포 가능합니다.

## 👨‍💻 개발자

ezid007

---

**마지막 수정:** 2025년 12월 9일
