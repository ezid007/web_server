#!/usr/bin/env python3
"""
YOLOv11n 파인튜닝 스크립트
수집된 데이터셋으로 모델을 학습시킵니다.
"""

import argparse
from pathlib import Path
from ultralytics import YOLO

# 프로젝트 경로 설정
PROJECT_DIR = Path(__file__).parent.parent
DATASET_DIR = PROJECT_DIR / "dataset"
MODELS_DIR = PROJECT_DIR / "models"
CLASSES_FILE = PROJECT_DIR / "classes.yaml"


def train_model(
    base_model: str = "yolo11n.pt",
    epochs: int = 100,
    imgsz: int = 640,
    batch: int = 16,
    device: str = "0",
    patience: int = 20
):
    """
    YOLOv11n 파인튜닝
    
    Args:
        base_model: 기본 모델 경로
        epochs: 학습 에폭 수
        imgsz: 입력 이미지 크기
        batch: 배치 크기
        device: 학습 디바이스 (0, 1, cpu 등)
        patience: 조기 종료 patience
    """
    # 모델 디렉토리 생성
    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    
    # 데이터셋 확인
    images = list((DATASET_DIR / "images").glob("*.jpg"))
    labels = list((DATASET_DIR / "labels").glob("*.txt"))
    
    print("\n" + "=" * 50)
    print("🚀 YOLOv11n 파인튜닝 시작")
    print("=" * 50)
    print(f"📁 데이터셋: {len(images)}개의 이미지, {len(labels)}개의 라벨")
    print(f"📋 클래스 정의: {CLASSES_FILE}")
    print(f"🔧 설정:")
    print(f"   - 기본 모델: {base_model}")
    print(f"   - 에폭: {epochs}")
    print(f"   - 이미지 크기: {imgsz}")
    print(f"   - 배치 크기: {batch}")
    print(f"   - 디바이스: {device}")
    print("=" * 50 + "\n")
    
    if len(images) < 10:
        print("⚠️ 경고: 이미지 수가 10개 미만입니다. 더 많은 데이터를 수집하세요!")
    
    # 모델 로드
    print(f"🔄 기본 모델 로딩: {base_model}")
    model = YOLO(base_model)
    
    # 학습 시작
    results = model.train(
        data=str(CLASSES_FILE),
        epochs=epochs,
        imgsz=imgsz,
        batch=batch,
        device=device,
        patience=patience,
        save=True,
        project=str(MODELS_DIR),
        name="user_detection",
        exist_ok=True,
        pretrained=True,
        verbose=True
    )
    
    # 결과 출력
    print("\n" + "=" * 50)
    print("✅ 학습 완료!")
    print("=" * 50)
    print(f"📁 모델 저장 위치: {MODELS_DIR / 'user_detection' / 'weights'}")
    print(f"   - best.pt: 최고 성능 모델")
    print(f"   - last.pt: 마지막 체크포인트")
    print("=" * 50 + "\n")
    
    # 검증
    print("🔍 모델 검증 중...")
    metrics = model.val()
    print(f"\n📊 검증 결과:")
    print(f"   - mAP50: {metrics.box.map50:.4f}")
    print(f"   - mAP50-95: {metrics.box.map:.4f}")
    
    return results


def main():
    parser = argparse.ArgumentParser(description="YOLOv11n 파인튜닝")
    parser.add_argument("--model", "-m", default="yolo11n.pt", help="기본 모델")
    parser.add_argument("--epochs", "-e", type=int, default=100, help="에폭 수")
    parser.add_argument("--imgsz", "-s", type=int, default=640, help="이미지 크기")
    parser.add_argument("--batch", "-b", type=int, default=16, help="배치 크기")
    parser.add_argument("--device", "-d", default="0", help="디바이스 (0, 1, cpu)")
    parser.add_argument("--patience", "-p", type=int, default=20, help="조기 종료 patience")
    
    args = parser.parse_args()
    
    train_model(
        base_model=args.model,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        patience=args.patience
    )


if __name__ == "__main__":
    main()
