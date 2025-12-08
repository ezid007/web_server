# Auto-Labeling 프로젝트
# YOLOv11n을 활용한 반자동 라벨링 시스템

## 프로젝트 구조
```
auto_labeling/
├── dataset/
│   ├── images/      # 캡처된 이미지
│   └── labels/      # YOLO 포맷 라벨 파일
├── models/          # 학습된 모델 저장
├── scripts/         # 실행 스크립트
├── classes.yaml     # 클래스 정의
└── requirements.txt
```

## 사용 방법
1. `pip install -r requirements.txt`
2. `python scripts/capture_and_label.py` - 이미지 캡처 및 자동 라벨링
3. `python scripts/review_labels.py` - 라벨 검토 및 클래스 수정
4. `python scripts/train.py` - YOLOv11n 파인튜닝
