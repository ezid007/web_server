# ROS2 네트워크 인터페이스 설정 가이드

## 개요

ROS2는 **DDS (Data Distribution Service)** 미들웨어를 사용하여 노드 간 통신을 수행합니다. 여러 네트워크 인터페이스(유선랜, 무선랜 등)가 있는 시스템에서 ROS2가 어떤 네트워크를 사용할지 지정하는 것을 **DDS 네트워크 인터페이스 설정** 또는 **RMW(ROS Middleware) 설정**이라고 합니다.

## 용어 정리

| 용어                    | 설명                                                        |
| ----------------------- | ----------------------------------------------------------- |
| **DDS**                 | Data Distribution Service. ROS2의 통신 미들웨어 표준        |
| **RMW**                 | ROS Middleware. DDS 구현체와 ROS2를 연결하는 추상화 계층    |
| **CycloneDDS**          | Eclipse 재단의 오픈소스 DDS 구현체 (ROS2 Humble 기본값)     |
| **FastDDS**             | eProsima의 DDS 구현체 (ROS2 Galactic 이전 기본값)           |
| **네트워크 인터페이스** | 물리적/가상 네트워크 장치 (예: eth0, wlan0, enp2s0, wlp3s0) |

## 현재 시스템 네트워크 구조

```
┌─────────────────────────────────────────────────────────────┐
│                        PC (A15)                              │
├─────────────────────────────────────────────────────────────┤
│  enp2s0 (유선랜)          │  wlp3s0 (무선랜)                 │
│  192.168.0.104            │  192.168.123.101                 │
│  ↓                        │  ↓                               │
│  일반 인터넷/통신용        │  ROS2 로봇 통신용                │
└─────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
                            ┌───────────────┐
                            │   TurtleBot3  │
                            │ 192.168.123.x │
                            └───────────────┘
```

## 설정 방법

### 1. 네트워크 인터페이스 이름 확인

```bash
# 방법 1: ip 명령어
ip addr show

# 방법 2: ifconfig (net-tools 필요)
ifconfig

# 방법 3: 간단히 인터페이스 목록만
ls /sys/class/net/
```

**출력 예시:**

```
1: lo: <LOOPBACK,UP,LOWER_UP>           # 루프백 (무시)
    inet 127.0.0.1/8
2: enp2s0: <BROADCAST,MULTICAST,UP>     # 유선랜
    inet 192.168.0.104/24
3: wlp3s0: <BROADCAST,MULTICAST,UP>     # 무선랜 ← ROS2용
    inet 192.168.123.101/24
```

### 2. CycloneDDS 설정 파일 생성

**파일 위치:** `/home/tuf/ros2_config/cyclonedds.xml`

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config"
            xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
            xsi:schemaLocation="https://cdds.io/config
            https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain>
        <General>
            <Interfaces>
                <!-- 사용할 네트워크 인터페이스 지정 -->
                <NetworkInterface name="wlp3s0"/>
            </Interfaces>
        </General>
    </Domain>
</CycloneDDS>
```

#### 설정 옵션 상세

**단일 인터페이스 지정:**

```xml
<Interfaces>
    <NetworkInterface name="wlp3s0"/>
</Interfaces>
```

**여러 인터페이스 지정:**

```xml
<Interfaces>
    <NetworkInterface name="wlp3s0"/>
    <NetworkInterface name="enp2s0"/>
</Interfaces>
```

**자동 선택 (권장하지 않음):**

```xml
<Interfaces>
    <NetworkInterface autodetermine="true"/>
</Interfaces>
```

**IP 주소로 지정:**

```xml
<Interfaces>
    <NetworkInterface address="192.168.123.101"/>
</Interfaces>
```

### 3. 환경변수 설정

**~/.bashrc에 추가:**

```bash
# ROS2 DDS 설정
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/tuf/ros2_config/cyclonedds.xml

# ROS2 Domain ID (같은 네트워크의 다른 ROS2 시스템과 분리)
export ROS_DOMAIN_ID=7
```

**적용:**

```bash
source ~/.bashrc
```

### 4. 설정 확인

```bash
# 환경변수 확인
echo $RMW_IMPLEMENTATION
echo $CYCLONEDDS_URI
echo $ROS_DOMAIN_ID

# ROS2 토픽 확인 (로봇이 연결되어 있을 때)
ros2 topic list

# 특정 토픽 데이터 확인
ros2 topic echo /battery_state
```

## 환경변수 상세 설명

### RMW_IMPLEMENTATION

ROS2에서 사용할 DDS 구현체를 지정합니다.

| 값                   | DDS 구현체  | 설명                            |
| -------------------- | ----------- | ------------------------------- |
| `rmw_cyclonedds_cpp` | CycloneDDS  | ROS2 Humble 기본값, 가볍고 빠름 |
| `rmw_fastrtps_cpp`   | FastDDS     | 이전 버전 기본값, 기능 풍부     |
| `rmw_connextdds`     | RTI Connext | 상용 DDS, 라이선스 필요         |

### CYCLONEDDS_URI

CycloneDDS 설정 파일 경로를 지정합니다.

```bash
# 파일 경로
export CYCLONEDDS_URI=file:///home/tuf/ros2_config/cyclonedds.xml

# 또는 직접 XML 지정 (간단한 설정)
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="wlp3s0"/></Interfaces></General></Domain></CycloneDDS>'
```

### ROS_DOMAIN_ID

같은 네트워크에서 여러 ROS2 시스템을 분리합니다.

-   범위: 0 ~ 232
-   같은 Domain ID를 가진 노드들만 서로 통신
-   기본값: 0

```bash
# 예: 로봇1은 Domain 7, 로봇2는 Domain 8
export ROS_DOMAIN_ID=7
```

## 문제 해결

### 토픽이 보이지 않을 때

1. **네트워크 인터페이스 확인**

    ```bash
    ip addr show wlp3s0
    ```

2. **CycloneDDS 설정 파일 확인**

    ```bash
    cat $CYCLONEDDS_URI | sed 's/file:\/\///'
    ```

3. **환경변수 확인**

    ```bash
    env | grep -E "RMW|CYCLONE|ROS_DOMAIN"
    ```

4. **로봇과 같은 네트워크인지 확인**

    ```bash
    ping 192.168.123.1  # 로봇 IP
    ```

5. **방화벽 확인**
    ```bash
    sudo ufw status
    # DDS 포트 열기 (필요시)
    sudo ufw allow 7400:7500/udp
    ```

### Deprecated 경고 해결

**이전 형식 (deprecated):**

```xml
<NetworkInterfaceAddress>wlp3s0</NetworkInterfaceAddress>
```

**새 형식 (권장):**

```xml
<Interfaces>
    <NetworkInterface name="wlp3s0"/>
</Interfaces>
```

### 여러 PC 간 통신 설정

모든 PC에서 동일하게 설정:

1. **같은 ROS_DOMAIN_ID 사용**
2. **같은 네트워크 대역 사용** (예: 192.168.123.x)
3. **각 PC의 CycloneDDS 설정에서 해당 네트워크 인터페이스 지정**

## 참고 자료

-   [CycloneDDS 공식 문서](https://cyclonedds.io/docs/)
-   [ROS2 DDS 튜닝 가이드](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html)
-   [ROS2 도메인 ID 설명](https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html)
