# 카메라 영상 압축 가이드

## 문제점
- 640x480 RGB888 영상은 프레임당 약 **921KB** (640 × 480 × 3 bytes)
- 30 FPS 기준 초당 약 **27.6 MB/s** 대역폭 필요
- 네트워크 대역폭 부족으로 영상 끊김 발생

## 해결 방법

### 방법 1: `image_transport` 사용 (추천)

ROS2의 `image_transport` 패키지를 사용하면 자동으로 압축/해제가 됩니다.

#### 설치
```bash
sudo apt install ros-humble-image-transport-plugins
```

#### 발행 측 (라즈베리파이)
`image_transport`가 설치되면 자동으로 압축 토픽이 생성됩니다:
- `/camera/image_raw` - 원본 (비압축)
- `/camera/image_raw/compressed` - JPEG 압축
- `/camera/image_raw/compressedDepth` - 깊이 이미지용
- `/camera/image_raw/theora` - Theora 비디오 압축

#### 수신 측 (원격 PC)
```bash
# 압축된 이미지 구독하여 자동 해제
ros2 run image_transport republish compressed raw \
  --ros-args -r in/compressed:=/camera/image_raw/compressed \
  -r out:=/camera/image_decompressed
```

또는 `rqt_image_view`에서 직접 압축 토픽 선택:
```bash
ros2 run rqt_image_view rqt_image_view
# 드롭다운에서 /camera/image_raw/compressed 선택
```

---

### 방법 2: JPEG 압축 품질 조정

압축 품질을 조정하여 대역폭 추가 절감:

```bash
# 압축 품질 파라미터 설정 (1-100, 기본값: 80)
ros2 param set /camera_node image_transport.compressed.jpeg_quality 50
```

품질별 대략적인 용량:
| 품질 | 프레임당 용량 | 30 FPS 대역폭 |
|------|---------------|---------------|
| 90   | ~50 KB        | ~1.5 MB/s     |
| 70   | ~30 KB        | ~900 KB/s     |
| 50   | ~20 KB        | ~600 KB/s     |
| 30   | ~10 KB        | ~300 KB/s     |

---

### 방법 3: 해상도 낮추기 + 압축

`robot_params.yaml`에서 해상도 조정:
```yaml
camera:
  width: 320      # 640 → 320
  height: 240     # 480 → 240
  fps: 15         # 30 → 15
```

---

## 권장 설정

### 라즈베리파이 (robot_params.yaml)
```yaml
camera:
  enabled: true
  format: "RGB888"
  width: 320          # 저해상도
  height: 240
  fps: 15             # 낮은 FPS
```

### 원격 PC에서 수신
```bash
# 1. image_transport 설치
sudo apt install ros-humble-image-transport-plugins

# 2. 압축된 이미지 확인
ros2 topic echo /camera/image_raw/compressed --no-arr

# 3. rqt로 보기 (자동 압축 해제)
ros2 run rqt_image_view rqt_image_view

# 4. 압축 해제하여 다른 토픽으로 발행
ros2 run image_transport republish compressed raw \
  --ros-args -r in/compressed:=/camera/image_raw/compressed \
  -r out:=/camera/image_decompressed
```

---

## Python 코드에서 압축 이미지 수신

```python
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

def compressed_callback(msg: CompressedImage):
    # JPEG 압축 해제
    np_arr = np.frombuffer(msg.data, np.uint8)
    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
    # 이미지 처리...
    cv2.imshow("Image", image)
    cv2.waitKey(1)

# 구독
node.create_subscription(
    CompressedImage,
    '/camera/image_raw/compressed',
    compressed_callback,
    10
)
```

---

## 대역폭 비교

| 설정 | 해상도 | FPS | 압축 | 대역폭 |
|------|--------|-----|------|--------|
| 원본 | 640x480 | 30 | 없음 | ~27 MB/s |
| 압축 | 640x480 | 30 | JPEG 50% | ~600 KB/s |
| 저해상도 | 320x240 | 30 | 없음 | ~6.9 MB/s |
| 저해상도+압축 | 320x240 | 15 | JPEG 50% | ~150 KB/s |

---

## 요약

1. **`ros-humble-image-transport-plugins` 설치** (양쪽 모두)
2. **발행 측**: 별도 설정 없이 자동으로 `/compressed` 토픽 생성
3. **수신 측**: `/camera/image_raw/compressed` 토픽 구독
4. **추가 최적화**: 해상도, FPS, JPEG 품질 조정
