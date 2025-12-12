# Web Server - AI 로봇 대시보드 프로젝트

TurtleBot3 제어 및 모니터링을 위한 **AI 통합 웹 기반 대시보드 서버**입니다.

## ✨ 주요 기능

-   🤖 **PHi-4 AI 챗봇:** QLoRA(4-bit) 최적화된 온디바이스 LLM, Google 검색 RAG 지원
-   👁️ **YOLO v11 객체 감지:** 실시간 카메라/CCTV 스트림 분석, 커스텀 Owner 인식
-   🛡️ **감시 모드 (Surveillance):** 시간 기반 자동 활성화, 소리 감지 연동
-   🐾 **펫 모드 (Pet Mode):** YOLO + LiDAR 센서 퓨전으로 Owner 추종
-   🗺️ **Nav2 자율주행:** 웹에서 목표점 클릭 설정, SLAM 맵 시각화
-   📹 **CCTV 프록싱:** 내부망 CCTV 스트림을 웹 대시보드로 안전하게 릴레이

## 📁 프로젝트 구조

```
web_server/
├── docs/                          # 📚 문서 모음
│   ├── 01_DEPLOYMENT_GUIDE.md     # 서버 배포 및 실행 가이드
│   ├── 02_ROS2_NETWORK_CONFIGURATION.md  # ROS2 네트워크 설정 가이드
│   ├── interview_prep.md          # 면접 준비 자료
│   └── self_introduction.md       # 자기소개서
├── Phi-4/                         # 🧠 PHi-4 LLM 파인튜닝
│   └── src/train/train.py         # QLoRA 학습 스크립트
├── auto_labeling/                 # 🏷️ YOLO 데이터 수집 및 라벨링
│   └── scripts/train.py           # YOLOv11n 파인튜닝 스크립트
├── detect/                        # 👁️ YOLO 감지 모듈
│   ├── yolo_detector.py           # 객체 감지 클래스
│   └── surveillance.py            # 감시 모드 로직
├── my_pet/                        # 🐾 펫 모드 모듈
│   └── pet_mode.py                # Owner 추종 로직
├── src/                           # 🔧 핵심 소스
│   ├── ros_node.py                # ROS2 노드 래퍼
│   ├── config.py                  # 환경 설정 로더
│   └── robotis/launch/            # Nav2 + Rosbridge Launch 파일
├── models/                        # 🧠 학습된 모델 저장소
│   └── my_yolo.pt                 # 커스텀 YOLO 모델
├── static/                        # 🎨 정적 파일 (CSS, JS, 이미지)
├── templates/                     # 📄 Jinja2 HTML 템플릿
├── main.py                        # 🚀 FastAPI 메인 서버
├── .env                           # 🔐 환경 변수 설정
└── yolo11n.pt                     # YOLO v11 Nano 기본 모델
```

## 🚀 빠른 시작

### 1. 환경 설정
```bash
cd /home/tuf/web_server
cp .env.example .env  # 필요 시 수정
poetry install
```

### 2. 서버 실행
```bash
poetry run python main.py
```
서버: `http://0.0.0.0:8000`

## 🛠 기술 스택

| 분류 | 기술 |
|------|------|
| **Backend** | FastAPI, Python 3.10, Poetry |
| **Frontend** | HTML/CSS/JS, GSAP, SwiperJS, Lottie |
| **AI/ML** | PHi-4 (QLoRA 4-bit), YOLO v11n, Ultralytics |
| **Robotics** | ROS2 Humble, Nav2, CycloneDDS, Rosbridge |
| **Database** | PostgreSQL |

## 💻 시스템 아키텍처

본 프로젝트는 **5개의 노드**와 **2개의 네트워크 대역**으로 구성된 분산 시스템입니다.

### 전체 구성도
```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                           🖥️ 중앙 통제 노트북 (Dual NIC)                           │
│                         ┌───────────────────────────────────────┐               │
│                         │   FastAPI Web Server (main.py)        │               │
│                         │   - PHi-4 LLM (QLoRA 4-bit)           │               │
│                         │   - YOLO v11 Inference                │               │
│                         │   - ROS2 Node (rclpy)                 │               │
│                         │   - CCTV Reverse Proxy                │               │
│                         └───────────────────────────────────────┘               │
│                                  │                   │                          │
│                    ┌─────────────┴───────┐   ┌───────┴───────────┐              │
│                    │ NIC1: 192.168.123.x │   │ NIC2: 192.168.0.x │              │
│                    └─────────────────────┘   └───────────────────┘              │
└────────────────────────────│───────────────────────────────────│────────────────┘
                             │                                   │
          ━━━━━━━━━━━━━━━━━━━┿━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┿━━━━━━━━━━━━━━━━━
          ┃     로봇 네트워크 (192.168.123.x)         ┃   CCTV 네트워크 (0.x)          ┃
          ━━━━━━━━━━━━━━━━━━━┿━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┿━━━━━━━━━━━━━━━━━
                             │                                   │
       ┌─────────────────────┼──────────────────────┐            │
       │                     │                      │            │
       ▼                     ▼                      │            ▼
┌──────────────────┐  ┌─────────────────────────┐   │  ┌─────────────────────────┐
│ 🤖 TurtleBot3    │  │ 🖥️ YOLO 학습 서버         │   │  │ 📹 CCTV 서버             │
│ (192.168.123.107)│  │ (192.168.123.103)       │   │  │ (192.168.0.197:8000)    │
│                  │  │                         │   │  │                         │
│ - ROS2 Humble    │  │ - GPU (RTX3060)         │   │  │ - 움직임 감지              │
│ - Camera Node    │  │ - YOLO v11 Training     │   │  │ - 소음 감지 (75dB↑)       │
│ - Nav2 Stack     │  │ - SSH/SCP 접속           │   │  │ - MJPEG Stream          │
│ - LiDAR (Scan)   │  │                         │   │  │ - DB 서버 연동            │
└──────┬───────────┘  └────────────┬────────────┘   │  └────────────┬────────────┘
       │                           │                │               │
       │  ROS2 DDS (CycloneDDS)    │  SSH/SCP (모델) │               │ HTTP (영상)
       └───────────────────────────┴────────────────┘               │
                                                                    ▼
                                                         ┌───────────────────────┐
                                                         │ 🗄️ PostgreSQL DB 서버  │
                                                         │ (192.168.0.192)       │
                                                         │ (75dB↑ 이벤트 파일 저장) │
                                                         └───────────────────────┘
```

### 노드별 역할

| 노드 | IP 주소 | 역할 | 통신 프로토콜 |
|------|---------|------|---------------|
| **중앙 통제 노트북** | - | 웹 서버, AI 추론, 로봇 제어 | HTTP, WebSocket, ROS2 DDS |
| **TurtleBot3** | 192.168.123.107 | 자율주행, 센서 데이터 제공 | ROS2 (CycloneDDS) |
| **YOLO 학습 서버** | 192.168.123.103 | GPU 학습 (Offline) | SSH/SCP |
| **CCTV 서버** | 192.168.0.197 | 움직임/소음 감지, 영상 스트림 | HTTP (MJPEG) |
| **DB 서버** | 192.168.0.192 | CCTV 이벤트 저장, 75dB↑ 파일 저장 | PostgreSQL |

### 네트워크 격리 전략
*   **ROS2 트래픽:** CycloneDDS XML 설정으로 **123.x 대역에만 바인딩**하여 멀티캐스트 간섭 방지.
*   **CCTV 프록시:** 0.x 내부망에 있는 CCTV 스트림을 FastAPI가 **리버스 프록시**로 중계하여 외부 접근 허용.

## ⚙️ 환경 변수 (.env)

| 변수 | 설명 |
|------|------|
| `TURTLEBOT_IP` | 로봇 IP 주소 |
| `CCTV_SERVER_URL` | CCTV 프록시 대상 URL |
| `YOLO_LABELING_MODEL` | YOLO 기본 모델 (yolo11n.pt) |
| `YOLO_OUTPUT_MODEL` | 학습 결과 저장 이름 |
| `GOOGLE_API_KEY` | RAG용 Google 검색 API 키 |

## 📚 문서

-   **[배포 가이드](docs/01_DEPLOYMENT_GUIDE.md)** - 설치, 실행, systemd 설정
-   **[ROS2 네트워크](docs/02_ROS2_NETWORK_CONFIGURATION.md)** - DDS/무선랜 설정


## 👨‍💻 개발자

ezid007

---
**마지막 수정:** 2025년 12월 12일
