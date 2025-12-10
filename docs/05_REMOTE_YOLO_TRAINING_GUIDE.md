# 🖥️ 다른 PC에서 YOLO 학습 가이드

GitHub로 프로젝트를 클론한 후 다른 PC에서 YOLO 학습을 실행하는 방법입니다.

---

## 📋 개요

GitHub `.gitignore`에 의해 제외되는 파일들을 수동으로 복사해야 합니다:

| 항목 | 경로 | 크기 (참고) |
|------|------|------------|
| 환경 설정 | `.env` | ~1KB |
| 학습 데이터셋 | `auto_labeling/dataset/` | ~130MB (1756장) |
| 기본 YOLO 모델 | `models/yolo11n.pt` | ~5.3MB |
| 클래스 정의 | `auto_labeling/classes.yaml` | ~100B |

---

## 🚀 Step 1: 학습 PC에서 준비

### 1-1. 프로젝트 클론
```bash
git clone https://github.com/YOUR_USERNAME/web_server.git
cd web_server
```

### 1-2. 필요한 폴더 생성
```bash
mkdir -p auto_labeling/dataset/images
mkdir -p auto_labeling/dataset/labels
mkdir -p models
```

### 1-3. Poetry 환경 설정
```bash
poetry install
```

---

## 📦 Step 2: 이 PC에서 파일 복사

USB, SCP, 또는 클라우드를 통해 다음 파일들을 복사합니다.

### 복사할 파일 목록

```bash
# 이 PC에서 실행 - 필요한 파일 압축
cd /home/tuf/web_server

# 옵션 1: 전체 압축 (권장)
tar -czvf yolo_training_files.tar.gz \
    .env \
    auto_labeling/classes.yaml \
    auto_labeling/dataset/ \
    models/yolo11n.pt

# 옵션 2: 개별 복사 (SCP 사용 시)
# scp .env user@학습PC:/path/to/web_server/
# scp auto_labeling/classes.yaml user@학습PC:/path/to/web_server/auto_labeling/
# scp -r auto_labeling/dataset user@학습PC:/path/to/web_server/auto_labeling/
# scp models/yolo11n.pt user@학습PC:/path/to/web_server/models/
```

### 학습 PC에서 압축 해제
```bash
cd /path/to/web_server
tar -xzvf yolo_training_files.tar.gz
```

---

## ⚙️ Step 3: 학습 PC에서 설정 확인

### 3-1. .env 파일 확인
학습 PC의 환경에 맞게 수정이 필요할 수 있습니다:
```bash
# .env 파일 확인
cat .env | grep YOLO
```

예상 출력:
```
YOLO_ENABLED=false
YOLO_CONFIDENCE=0.7
YOLO_DASHBOARD_MODEL=my_yolo.pt
YOLO_LABELING_MODEL=yolo11n.pt
YOLO_OUTPUT_MODEL=my_yolo.pt
```

### 3-2. 데이터셋 확인
```bash
ls auto_labeling/dataset/images/ | wc -l  # 이미지 수
ls auto_labeling/dataset/labels/ | wc -l  # 라벨 수
```

### 3-3. GPU 확인 (선택사항)
```bash
nvidia-smi  # NVIDIA GPU 확인
```

---

## 🎓 Step 4: 학습 실행

### 기본 학습
```bash
cd /path/to/web_server/auto_labeling
poetry run python scripts/train.py --epochs 50
```

### GPU 메모리 부족 시
```bash
poetry run python scripts/train.py --epochs 50 --batch 8
```

### CPU 사용 시 (느림)
```bash
poetry run python scripts/train.py --epochs 50 --device cpu
```

---

## 📥 Step 5: 학습된 모델 가져오기

학습 완료 후 다음 파일을 이 PC로 복사합니다:

```bash
# 학습 PC에서 실행
# 최종 모델 위치: /path/to/web_server/models/my_yolo.pt

# 옵션 1: SCP로 복사
scp /path/to/web_server/models/my_yolo.pt tuf@이PC의IP:/home/tuf/web_server/models/

# 옵션 2: USB로 복사
cp /path/to/web_server/models/my_yolo.pt /media/usb/
```

### 이 PC에서 복사 받기
```bash
# USB에서 복사
cp /media/usb/my_yolo.pt /home/tuf/web_server/models/

# 또는 학습 PC에서 직접 가져오기
scp user@학습PC:/path/to/web_server/models/my_yolo.pt /home/tuf/web_server/models/
```

---

## ✅ Step 6: 모델 적용 확인

### 대시보드에서 사용
`.env` 파일에서 모델이 설정되어 있는지 확인:
```bash
# .env
YOLO_DASHBOARD_MODEL=my_yolo.pt
```

### 웹 서버 재시작
```bash
cd /home/tuf/web_server
poetry run python main.py
```

---

## 📁 파일 체크리스트

### 복사해야 할 파일 (이 PC → 학습 PC)

| 체크 | 파일/폴더 | 설명 |
|------|----------|------|
| ☐ | `.env` | 환경 설정 (YOLO_OUTPUT_MODEL 포함) |
| ☐ | `auto_labeling/classes.yaml` | 클래스 정의 |
| ☐ | `auto_labeling/dataset/images/` | 학습 이미지 |
| ☐ | `auto_labeling/dataset/labels/` | 라벨 파일 |
| ☐ | `models/yolo11n.pt` | 기본 YOLO 모델 |

### 가져와야 할 파일 (학습 PC → 이 PC)

| 체크 | 파일 | 설명 |
|------|------|------|
| ☐ | `models/my_yolo.pt` | 학습된 모델 |

---

## ⚠️ 주의사항

1. **GPU 권장**: CPU 학습은 매우 느립니다 (10배 이상 차이)
2. **데이터셋 무결성**: 이미지와 라벨 파일 수가 동일해야 합니다
3. **모델 이름**: `.env`의 `YOLO_OUTPUT_MODEL`과 `YOLO_DASHBOARD_MODEL`이 일치하는지 확인
4. **Poetry 버전**: 양쪽 PC의 Python/Poetry 버전이 호환되어야 합니다

---

## 🔧 문제 해결

### "No module named 'ultralytics'" 오류
```bash
poetry install  # 의존성 재설치
```

### GPU 메모리 부족
```bash
# 배치 크기 줄이기
poetry run python scripts/train.py --batch 4
```

### 모델 파일을 찾을 수 없음
```bash
# 모델 경로 확인
ls -la models/
# .env 설정 확인
cat .env | grep YOLO
```
