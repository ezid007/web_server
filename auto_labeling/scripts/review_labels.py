#!/usr/bin/env python3
"""
라벨 검토 및 수정 스크립트
저장된 이미지와 라벨을 검토하고 클래스 ID를 수정합니다.
"""

import os
import cv2
import yaml
from pathlib import Path

# 프로젝트 경로 설정
PROJECT_DIR = Path(__file__).parent.parent
DATASET_DIR = PROJECT_DIR / "dataset"
IMAGES_DIR = DATASET_DIR / "images"
LABELS_DIR = DATASET_DIR / "labels"
CLASSES_FILE = PROJECT_DIR / "classes.yaml"


def load_classes():
    """classes.yaml에서 클래스 목록 로드"""
    if CLASSES_FILE.exists():
        with open(CLASSES_FILE, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
            return data.get('names', ['unknown'])
    return ['unknown']


def load_label(label_path: Path):
    """라벨 파일 로드"""
    labels = []
    if label_path.exists():
        with open(label_path, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) >= 5:
                    labels.append({
                        'class_id': int(parts[0]),
                        'x_center': float(parts[1]),
                        'y_center': float(parts[2]),
                        'width': float(parts[3]),
                        'height': float(parts[4])
                    })
    return labels


def save_label(label_path: Path, labels: list):
    """라벨 파일 저장"""
    with open(label_path, 'w') as f:
        for label in labels:
            line = f"{label['class_id']} {label['x_center']:.6f} {label['y_center']:.6f} {label['width']:.6f} {label['height']:.6f}\n"
            f.write(line)


def draw_labels(image, labels, classes, selected_idx=0):
    """이미지에 라벨 그리기"""
    h, w = image.shape[:2]
    display = image.copy()
    
    for idx, label in enumerate(labels):
        # 바운딩 박스 좌표 계산
        x_center = label['x_center'] * w
        y_center = label['y_center'] * h
        box_w = label['width'] * w
        box_h = label['height'] * h
        
        x1 = int(x_center - box_w / 2)
        y1 = int(y_center - box_h / 2)
        x2 = int(x_center + box_w / 2)
        y2 = int(y_center + box_h / 2)
        
        # 선택된 박스는 다른 색상
        color = (0, 255, 255) if idx == selected_idx else (0, 255, 0)
        thickness = 3 if idx == selected_idx else 2
        
        cv2.rectangle(display, (x1, y1), (x2, y2), color, thickness)
        
        # 클래스 이름 표시
        class_name = classes[label['class_id']] if label['class_id'] < len(classes) else f"ID:{label['class_id']}"
        cv2.putText(display, f"{class_name}", (x1, y1 - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    
    return display


def review_labels():
    """라벨 검토 및 수정 UI"""
    classes = load_classes()
    
    # 이미지 파일 목록
    image_files = sorted(IMAGES_DIR.glob("*.jpg"))
    
    if not image_files:
        print("❌ 이미지 파일이 없습니다!")
        return
    
    print("\n" + "=" * 50)
    print("🔍 라벨 검토 모드")
    print("=" * 50)
    print("조작 방법:")
    print("  [←/→] - 이전/다음 이미지")
    print("  [↑/↓] - 이전/다음 바운딩 박스 선택")
    print("  [1-9] - 선택된 박스의 클래스 변경")
    print("  [D]   - 선택된 박스 삭제")
    print("  [S]   - 변경사항 저장")
    print("  [Q]   - 종료")
    print("=" * 50)
    print(f"📋 클래스 목록: {classes}")
    print(f"📁 총 {len(image_files)}개의 이미지")
    print("=" * 50 + "\n")
    
    current_idx = 0
    selected_box_idx = 0
    modified = False
    
    while True:
        # 현재 이미지 및 라벨 로드
        image_path = image_files[current_idx]
        label_path = LABELS_DIR / image_path.with_suffix('.txt').name
        
        image = cv2.imread(str(image_path))
        labels = load_label(label_path)
        
        # 선택 인덱스 범위 조정
        if selected_box_idx >= len(labels):
            selected_box_idx = max(0, len(labels) - 1)
        
        # 이미지에 라벨 그리기
        display = draw_labels(image, labels, classes, selected_box_idx)
        
        # 상태 표시
        status = f"[{current_idx + 1}/{len(image_files)}] {image_path.name}"
        if modified:
            status += " [수정됨]"
        cv2.putText(display, status, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # 선택된 박스 정보
        if labels and selected_box_idx < len(labels):
            box_info = f"Box {selected_box_idx + 1}/{len(labels)}: {classes[labels[selected_box_idx]['class_id']]}"
            cv2.putText(display, box_info, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        cv2.imshow("Label Review (Press Q to quit)", display)
        
        key = cv2.waitKey(0) & 0xFF
        
        if key == ord('q'):
            if modified:
                print("⚠️ 저장되지 않은 변경사항이 있습니다!")
            break
        
        elif key == 83 or key == ord('d'):  # 오른쪽 화살표
            if modified and labels:
                save_label(label_path, labels)
                print(f"💾 저장됨: {label_path.name}")
            current_idx = min(current_idx + 1, len(image_files) - 1)
            selected_box_idx = 0
            modified = False
        
        elif key == 81 or key == ord('a'):  # 왼쪽 화살표
            if modified and labels:
                save_label(label_path, labels)
                print(f"💾 저장됨: {label_path.name}")
            current_idx = max(current_idx - 1, 0)
            selected_box_idx = 0
            modified = False
        
        elif key == 82:  # 위쪽 화살표
            selected_box_idx = max(0, selected_box_idx - 1)
        
        elif key == 84:  # 아래쪽 화살표
            selected_box_idx = min(len(labels) - 1, selected_box_idx + 1) if labels else 0
        
        elif ord('1') <= key <= ord('9'):  # 클래스 변경
            new_class_id = key - ord('1')
            if new_class_id < len(classes) and labels and selected_box_idx < len(labels):
                labels[selected_box_idx]['class_id'] = new_class_id
                modified = True
                print(f"🏷️  클래스 변경: {classes[new_class_id]}")
        
        elif key == ord('d'):  # 삭제
            if labels and selected_box_idx < len(labels):
                del labels[selected_box_idx]
                modified = True
                print("🗑️  박스 삭제됨")
        
        elif key == ord('s'):  # 저장
            if labels:
                save_label(label_path, labels)
                modified = False
                print(f"💾 저장됨: {label_path.name}")
    
    cv2.destroyAllWindows()
    print("\n✅ 라벨 검토 완료!")


if __name__ == "__main__":
    review_labels()
