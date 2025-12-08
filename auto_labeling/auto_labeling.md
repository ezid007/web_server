# 🤖 Auto-Labeling 사용 설명서

TurtleBot3 카메라를 활용한 반자동 데이터 수집 및 라벨링 시스템

---

## 📋 사전 준비

### 1. 환경 설정
```bash
cd /home/tuf/web_server
poetry install
```

### 2. TurtleBot3 준비
- TurtleBot3 전원 ON
- 카메라 노드 실행 확인
- 대시보드에서 카메라 스트림이 보이는지 확인

### 3. 카메라 스트림 URL 확인
- 로컬 web_server 실행 후 자동으로 연결됨
- 대시보드에서 카메라 화면이 나오면 스트림이 정상 작동 중

---

## 🎯 데이터 수집 방법

### Step 1: 새로운 사람 클래스 추가
```bash
# 본인 이름으로 클래스 추가
python scripts/capture_and_label.py --add-person "홍길동"

# 여러 사람 추가 가능
python scripts/capture_and_label.py --add-person "김철수"
```

### Step 2: 데이터 수집 시작
```bash
# 본인 데이터 수집 (클래스 ID 1 = 첫번째 추가한 사람)
python scripts/capture_and_label.py --class-id 1
```

### Step 3: 캡처 화면 조작법

| 키 | 기능 |
|---|---|
| `SPACE` | 현재 프레임 수동 저장 (사람 탐지 시) |
| `B` | 배경 이미지 저장 (빈 라벨 파일 생성) |
| `A` | 자동 저장 모드 ON/OFF (1초 간격) |
| `N` | 새로운 사람 클래스 추가 |
| `1-9` | 클래스 ID 변경 |
| `Q` | 종료 |

### Step 4: 효과적인 데이터 수집 팁

**⚠️ 중요: 올바른 데이터 수집 방법**

1. **본인 이미지 (class 1)**: 정면, 측면, 후면 다양하게 촬영
2. **다른 사람 이미지 (class 2+)**: 다른 사람들도 수집하여 구분 학습
3. **배경 이미지 (B키)**: 사람 없는 빈 배경도 반드시 수집

**데이터 수집 가이드:**
- 다양한 거리: 0.5m ~ 3m
- 다양한 각도: 정면, 측면, 후면 (펫 로봇은 뒷모습 인식 필수!)
- 다양한 자세: 서있기, 앉기, 걷기
- 최소 수량: 한 사람당 200~500장, 배경 50~100장

---

## 🔍 라벨 검토 및 수정

### 라벨 검토 도구 실행
```bash
python scripts/review_labels.py
```

### 조작법

| 키 | 기능 |
|---|---|
| `←/→` (또는 `A/D`) | 이전/다음 이미지 |
| `↑/↓` | 바운딩 박스 선택 |
| `1-9` | 선택된 박스의 클래스 변경 |
| `D` | 선택된 박스 삭제 |
| `S` | 저장 |
| `Q` | 종료 |

---

## 🚀 모델 학습

### 데이터 수집 완료 후 학습 시작
```bash
# 기본 설정으로 학습 (GPU 사용)
python scripts/train.py

# CPU로 학습 (느림)
python scripts/train.py --device cpu

# 커스텀 설정
python scripts/train.py --epochs 50 --batch 8 --imgsz 640
```

### 학습 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `--epochs` | 100 | 학습 에폭 수 |
| `--batch` | 16 | 배치 크기 (GPU 메모리에 따라 조정) |
| `--imgsz` | 640 | 입력 이미지 크기 |
| `--device` | 0 | GPU ID (cpu 가능) |

---

## 📁 파일 구조

```
auto_labeling/
├── dataset/
│   ├── images/     ← 캡처된 이미지 저장 위치
│   └── labels/     ← YOLO 포맷 라벨 저장 위치
├── models/         ← 학습된 모델 저장 위치
│   └── user_detection/
│       └── weights/
│           ├── best.pt   ← 최고 성능 모델
│           └── last.pt   ← 마지막 체크포인트
├── scripts/
│   ├── capture_and_label.py
│   ├── review_labels.py
│   └── train.py
├── classes.yaml    ← 클래스 정의
└── auto_labeling.md
```

---

## ⚠️ 문제 해결

### 카메라 스트림 연결 실패
- web_server가 실행 중인지 확인
- `.env` 파일에 올바른 IP 설정 확인

### YOLO 모델 다운로드 오류
```bash
# 수동 다운로드
pip install ultralytics --upgrade
```

### GPU 메모리 부족
```bash
# 배치 크기 줄이기
python scripts/train.py --batch 4
```

---

## 📊 권장 데이터 수집량

| 클래스 수 | 클래스당 이미지 | 배경 이미지 | 총 이미지 |
|----------|----------------|------------|----------|
| 1명 | 300~500장 | 50~100장 | 350~600장 |
| 2명 | 200~300장 | 50~100장 | 450~700장 |
| 3명+ | 150~200장 | 50~100장 | 500~700장+ |

---

## 🎬 빠른 시작 요약

```bash
# 1. 환경 준비
cd /home/tuf/web_server
poetry install

# 2. 본인 클래스 추가
python scripts/capture_and_label.py --add-person "내이름"

# 3. 데이터 수집
# - 본인: class-id 1로 촬영
# - 다른 사람: class-id 2로 촬영
# - 배경: B키로 저장
python scripts/capture_and_label.py --class-id 1

# 4. 라벨 검토 (선택사항)
python scripts/review_labels.py

# 5. 모델 학습
python scripts/train.py --epochs 50
```
