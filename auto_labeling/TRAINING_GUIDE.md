# 🎓 YOLO 모델 학습 가이드

## 📋 사전 준비

### 1. 데이터 수집 완료 확인
```bash
# 이미지 수량 확인
ls auto_labeling/dataset/images/ | wc -l

# 라벨 수량 확인
ls auto_labeling/dataset/labels/ | wc -l
```

> 권장: 클래스당 200~500장

---

## 🚀 학습 실행

### 기본 학습
```bash
cd /home/tuf/web_server/auto_labeling
poetry run python scripts/train.py --epochs 50
```

### 커스텀 설정
```bash
# GPU 사용 (기본)
poetry run python scripts/train.py --epochs 100 --batch 16 --imgsz 640

# CPU 사용 (느림)
poetry run python scripts/train.py --epochs 50 --device cpu

# 작은 배치 (GPU 메모리 부족 시)
poetry run python scripts/train.py --epochs 50 --batch 8
```

---

## ⚙️ 학습 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `--epochs` | 100 | 학습 반복 횟수 |
| `--batch` | 16 | 배치 크기 (GPU 메모리에 따라 조정) |
| `--imgsz` | 640 | 입력 이미지 크기 |
| `--device` | 0 | GPU ID (cpu 가능) |
| `--patience` | 20 | 조기 종료 patience |

---

## 📁 학습 결과 위치

```
auto_labeling/models/user_detection/weights/
├── best.pt   ← 최고 성능 모델 (사용 권장)
└── last.pt   ← 마지막 체크포인트
```

---

## 🔍 학습 후 테스트

### 사용자 추적 실행
```bash
cd /home/tuf/web_server/auto_labeling
poetry run python scripts/user_follower.py --class-id 1
```

---

## 💡 팁

1. **학습 중 GPU 사용률 확인**: `nvidia-smi`
2. **메모리 부족 시**: `--batch 8` 또는 `--batch 4`로 줄이기
3. **더 정확한 모델**: `--epochs 100` 이상, 데이터 500장+
