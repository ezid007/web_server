# Dashboard CSS 스타일 가이드

이 문서는 `static/css/dashboard.css` 파일의 주요 섹션과 수정 방법을 설명합니다.

## 📁 파일 구조

```
static/css/dashboard.css (1448줄)
├── CSS 변수 (1-37줄)
├── 전역 리셋 (39-43줄)
├── 대시보드 페이지 (45-50줄)
├── 헤더 스타일 (52-128줄)
├── 메인 그리드 레이아웃 (130-156줄)
├── 카드 기본 스타일 (158-193줄)
├── ENERGY FLOW 섹션 (310-515줄)
├── ROBOT CARD 섹션 (517-620줄)
├── DETAILS 카드 (880-920줄)
├── CABLE 카드 (922-1075줄)
├── VOLTAGE 카드 (1077-1174줄)
├── CHARGING 카드 (1176-1260줄)
└── 반응형 미디어 쿼리 (1262-1295줄)
```

## 🎨 CSS 변수 (색상 변경)

**위치**: 1-37줄

```css
:root {
    /* 배경색 */
    --bg-primary: #1a1d23; /* 메인 배경색 */
    --bg-secondary: #232831; /* 보조 배경색 (카드 내부) */
    --bg-card: #1e2128; /* 카드 배경색 */

    /* 강조색 */
    --accent-orange: #ff6b35; /* 주황색 - 버튼, 아이콘, 포인트 색상 */
    --accent-orange-glow: rgba(255, 107, 53, 0.3); /* 글로우 효과 */

    /* 텍스트 색상 */
    --text-primary: #ffffff; /* 주요 텍스트 (흰색) */
    --text-secondary: #8b8e98; /* 보조 텍스트 (회색) */
    --text-dim: #5a5d66; /* 흐린 텍스트 (어두운 회색) */

    /* 기타 */
    --border-color: #2d3139; /* 테두리 색상 */
    --success: #4caf50; /* 성공 상태 (녹색) */
    --warning: #ff9800; /* 경고 상태 (주황색) */
}
```

**수정 방법**: 색상 코드를 변경하면 전체 대시보드 색상이 변경됩니다.

---

## 📐 메인 그리드 레이아웃

**위치**: 130-156줄

```css
.dashboard-page .dashboard-main {
    display: grid;
    grid-template-columns: repeat(3, 1fr); /* 3개의 동일한 너비 열 */
    gap: 1.5rem; /* 카드 간 간격 */
    padding: 2rem; /* 외부 여백 */
    max-width: 1800px; /* 최대 너비 */
    margin: 14rem auto 4rem; /* 상단 14rem, 좌우 자동(중앙정렬), 하단 4rem */
    min-height: 100vh; /* 최소 높이 100vh */
}
```

### 그리드 레이아웃 구조:

```
┌─────────────────┬─────────────────┬─────────────────┐
│                 │                 │   ROBOT CARD    │
│  ENERGY FLOW    │  ENERGY FLOW    │   (카메라)       │
│  (SLAM 지도)     │  (포트 리스트)   │   row: 1-2      │
│  grid-column:   │                 │─────────────────│
│  1 / 3 (2칸)    │                 │   DETAILS       │
│  grid-row: 1-2  │                 │   row: 3        │
├─────────────────┼─────────────────┼─────────────────┤
│   CABLE CARD    │  VOLTAGE CARD   │  (위 계속)       │
│   row: 3        │   row: 3        │                 │
└─────────────────┴─────────────────┴─────────────────┘
│          CHARGING CARD (4행, 1-2열 span)            │
└─────────────────────────────────────────────────────┘
```

**수정 방법**:

-   `grid-template-columns`: 열 개수 변경 (예: `repeat(4, 1fr)` → 4열)
-   `gap`: 카드 간 간격 조정
-   `margin`: 상하좌우 여백 조정

---

## 🗺️ ENERGY FLOW 섹션 (SLAM 지도 + 포트 리스트)

**위치**: 310-515줄

### 주요 클래스:

#### `.energy-flow` - 전체 컨테이너

```css
.energy-flow {
    grid-column: 1 / 3; /* 1열부터 3열 전까지 (2칸 차지) */
    grid-row: 1 / 3; /* 1행부터 3행 전까지 (2행 차지) */
}
```

#### `.energy-content` - 지도/포트 그리드

```css
.energy-content {
    display: grid;
    grid-template-columns: 1fr 1fr; /* 지도(왼쪽) + 포트리스트(오른쪽) */
    gap: 1.5rem;
    margin-bottom: 1.5rem;
}
```

#### `.slam-map-view` - SLAM 지도 영역

```css
.slam-map-view {
    position: relative;
    height: 350px; /* 지도 높이 - 여기서 조정 가능 */
    background: var(--bg-secondary);
    border-radius: 0.5rem;
    overflow: hidden;
    border: 2px solid var(--border-color);
}
```

#### `.port-list` - 포트 리스트 컨테이너

```css
.port-list {
    display: flex;
    flex-direction: column; /* 세로 배치 */
    gap: 0.75rem;
}
```

#### `.port-item` - 개별 포트 아이템

```css
.port-item {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 1rem 1.25rem;
    background: var(--bg-secondary);
    border: 1px solid var(--border-color);
    border-radius: 0.5rem;
    transition: all 0.3s;
}

.port-item.active {
    border-color: var(--accent-orange); /* 활성화 시 주황색 테두리 */
    background: rgba(255, 107, 53, 0.05); /* 반투명 주황색 배경 */
}
```

#### `.port-toggle` - 토글 스위치

```css
.port-toggle {
    width: 40px;
    height: 24px;
    background: var(--bg-primary);
    border: 2px solid var(--border-color);
    border-radius: 12px;
    position: relative;
    cursor: pointer;
    transition: all 0.3s;
}

.port-toggle.on {
    background: var(--accent-orange); /* ON 상태: 주황색 */
    border-color: var(--accent-orange);
}

.port-toggle::after {
    content: "";
    position: absolute;
    width: 16px;
    height: 16px;
    background: var(--text-dim); /* OFF: 회색 */
    border-radius: 50%;
    top: 2px;
    left: 2px;
    transition: all 0.3s;
}

.port-toggle.on::after {
    left: 18px; /* ON 상태: 오른쪽으로 이동 */
    background: white; /* ON 상태: 흰색 */
}
```

#### `.battery-info` - 배터리 정보 하단 바

```css
.battery-info {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 1.5rem;
    background: var(--bg-secondary);
    border-radius: 0.5rem;
}

.time-value {
    font-size: 3rem; /* "12" 숫자 크기 */
    font-weight: 700;
}

.time-label {
    font-size: 1rem; /* "HR" 레이블 크기 */
    font-weight: 600;
}

.load-label {
    font-size: 0.75rem; /* "MEDIUM LOAD" 크기 */
    color: var(--accent-orange);
}
```

---

## 📷 ROBOT CARD 섹션 (TurtleBot3 카메라)

**위치**: 517-620줄

### 주요 클래스:

#### `.robot-card` - 전체 컨테이너

```css
.robot-card {
    grid-column: 3; /* 3열에 위치 */
    grid-row: 1 / 3; /* 1-2행 차지 (세로로 길게) */
}
```

#### `.robot-header` - 타이틀 영역

```css
.robot-header {
    margin-bottom: 1.5rem;
    text-align: center;
    padding: 1rem 0;
    border-bottom: 1px solid var(--border-color);
}
```

#### `.robot-title` - "TURTLEBOT-3" 제목

```css
.robot-title {
    font-size: 1.5rem; /* 제목 크기 */
    font-weight: 700;
    letter-spacing: 4px; /* 글자 간격 */
    color: var(--text-primary);
    text-align: center;
    background: linear-gradient(
        135deg,
        var(--accent-orange),
        #ff8555
    ); /* 그라디언트 */
    -webkit-background-clip: text;
    -webkit-text-fill-color: transparent;
    background-clip: text;
}
```

#### `.camera-view` - 카메라 영역

```css
.camera-view {
    position: relative;
    height: 360px; /* 카메라 높이 - 여기서 조정 */
    background: var(--bg-secondary);
    border-radius: 0.5rem;
    display: flex;
    align-items: center;
    justify-content: center;
    overflow: hidden;
    border: 2px solid var(--border-color);
    margin-bottom: 1.5rem;
}

.camera-view img {
    width: 100%;
    height: 100%;
    object-fit: cover; /* 이미지 채우기 방식 */
    background: #000;
}
```

#### `.camera-label` - 카메라 하단 라벨

```css
.camera-label {
    position: absolute;
    bottom: 0;
    left: 0;
    right: 0;
    font-size: 0.75rem;
    font-weight: 700;
    letter-spacing: 2px;
    color: var(--text-primary);
    background: linear-gradient(
        180deg,
        transparent,
        rgba(26, 29, 35, 0.95)
    ); /* 그라디언트 배경 */
    padding: 1.5rem 1rem 1rem;
    text-align: center;
    backdrop-filter: blur(10px); /* 블러 효과 */
}
```

#### `.power-control` - 전원 컨트롤 영역

```css
.power-control {
    display: flex;
    align-items: center;
    justify-content: center; /* 중앙 정렬 */
    gap: 1.5rem; /* 요소 간 간격 */
    padding: 1.25rem 1.5rem;
    background: var(--bg-secondary);
    border-radius: 0.5rem;
}
```

#### `.power-btn` - 전원 버튼

```css
.power-btn {
    width: 56px; /* 버튼 크기 */
    height: 56px;
    border-radius: 50%; /* 원형 */
    border: 3px solid var(--border-color);
    background: var(--bg-secondary);
    color: var(--text-dim); /* OFF 상태: 어두운 아이콘 */
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
    font-size: 1.6rem; /* 아이콘 크기 */
    transition: all 0.3s;
}

.power-btn.active {
    background: var(--accent-orange); /* ON 상태: 주황색 */
    color: white;
    border-color: var(--accent-orange);
    box-shadow: 0 0 25px var(--accent-orange-glow); /* 글로우 효과 */
}

.power-btn:hover {
    transform: scale(1.08); /* 호버 시 크기 증가 */
}
```

#### `.power-status` - "ON OFF" 텍스트

```css
.power-status {
    font-size: 0.95rem;
    font-weight: 700;
    letter-spacing: 2.5px;
    color: var(--text-primary);
}

.status-off {
    color: var(--text-dim); /* OFF 텍스트는 어두운 회색 */
    margin-left: 0.5rem;
}
```

---

## 📊 DETAILS 카드

**위치**: 880-920줄

```css
.details-card {
    grid-column: 3; /* 3열 */
    grid-row: 3; /* 3행 */
}

.detail-grid {
    display: grid;
    grid-template-columns: repeat(2, 1fr); /* 2열 그리드 */
    gap: 1rem;
}

.detail-item {
    display: flex;
    flex-direction: column;
    gap: 0.5rem;
    padding: 1rem;
    background: var(--bg-secondary);
    border-radius: 0.5rem;
}

.detail-label {
    font-size: 0.75rem;
    color: var(--text-secondary);
    letter-spacing: 1px;
}

.detail-value {
    font-size: 1rem;
    font-weight: 600;
    display: flex;
    align-items: center;
    gap: 0.5rem;
}

.detail-value i {
    color: var(--accent-orange); /* 아이콘 주황색 */
}
```

---

## ⚡ CABLE 카드 (FLASH MODE)

**위치**: 922-1075줄

```css
.cable-card {
    grid-column: 1; /* 1열 */
    grid-row: 3; /* 3행 */
}

.cable-visual {
    position: relative;
    height: 150px;
    background: var(--bg-secondary);
    border-radius: 0.5rem;
    display: flex;
    align-items: center;
    justify-content: center;
}

.time-remain {
    font-size: 1.5rem; /* "0 15" 숫자 크기 */
    font-weight: 700;
    letter-spacing: 2px;
}

.toggle-slider {
    width: 50px;
    height: 26px;
    background: var(--accent-orange); /* ON 상태 주황색 */
    border-radius: 13px;
    position: relative;
    cursor: pointer;
}
```

---

## 🔋 VOLTAGE 카드

**위치**: 1077-1174줄

```css
.voltage-card {
    grid-column: 2; /* 2열 */
    grid-row: 3; /* 3행 */
}

.voltage-meter {
    display: flex;
    align-items: center;
    gap: 1.5rem;
}

.meter-bars {
    display: flex;
    gap: 0.5rem;
}

.bar {
    width: 12px;
    height: 60px;
    background: var(--bg-primary);
    border-radius: 6px;
    transition: background 0.3s;
}

.bar.active {
    background: linear-gradient(
        180deg,
        var(--accent-orange),
        var(--warning)
    ); /* 주황색 그라디언트 */
    box-shadow: 0 0 10px var(--accent-orange-glow);
}

.voltage-value {
    font-size: 2.5rem; /* "23.8" 숫자 크기 */
    font-weight: 700;
}
```

---

## 🔌 CHARGING 카드

**위치**: 1176-1260줄

```css
.charging-card {
    grid-column: 1 / 3; /* 1-2열 (2칸 차지) */
    grid-row: 4; /* 4행 */
}

.charging-circle {
    position: relative;
    width: 200px;
    height: 200px;
    margin-bottom: 1rem;
}

.progress-ring-fill {
    fill: none;
    stroke: var(--accent-orange); /* 진행률 링 색상 */
    stroke-width: 12;
    stroke-linecap: round;
}

.percent-value {
    font-size: 2rem; /* "74%" 크기 */
    font-weight: 700;
}
```

---

## 📱 반응형 (모바일/태블릿)

**위치**: 1262-1295줄

```css
/* 1400px 이하: 2열 레이아웃 */
@media (max-width: 1400px) {
    .dashboard-main {
        grid-template-columns: repeat(2, 1fr);
    }

    .energy-flow {
        grid-column: span 2; /* 2칸 모두 차지 */
    }
}

/* 768px 이하: 1열 레이아웃 (모바일) */
@media (max-width: 768px) {
    .dashboard-main {
        grid-template-columns: 1fr;
        padding: 1rem;
    }

    .energy-flow,
    .robot-card,
    .charging-card {
        grid-column: span 1;
        grid-row: span 1;
    }
}
```

---

## 🔧 자주 수정하는 값들

### 색상 변경

-   **주황색 변경**: `:root` → `--accent-orange` (1번째 줄)
-   **배경색 변경**: `:root` → `--bg-primary`, `--bg-secondary` (1번째 줄)

### 크기 조정

-   **카메라 크기**: `.camera-view` → `height: 360px` (531번째 줄)
-   **SLAM 지도 크기**: `.slam-map-view` → `height: 350px` (334번째 줄)
-   **전원 버튼 크기**: `.power-btn` → `width: 56px; height: 56px` (577번째 줄)
-   **카드 간격**: `.dashboard-main` → `gap: 1.5rem` (133번째 줄)

### 레이아웃 조정

-   **그리드 열 개수**: `.dashboard-main` → `grid-template-columns: repeat(3, 1fr)` (132번째 줄)
-   **카드 위치**: 각 카드의 `grid-column`과 `grid-row` 속성

### 폰트 크기

-   **제목 크기**: `.robot-title` → `font-size: 1.5rem` (512번째 줄)
-   **숫자 크기**: `.time-value` → `font-size: 3rem` (462번째 줄)

---

## 💡 팁

1. **브라우저 캐시 문제**: CSS 수정 후 변경사항이 안 보이면

    - `dashboard.html`에서 `?v=숫자` 버전 올리기
    - 또는 브라우저에서 `Ctrl + Shift + R` (강제 새로고침)

2. **색상 통일**: CSS 변수를 사용하므로 `:root`에서만 색상을 변경하면 전체에 적용됩니다.

3. **반응형 확인**: 브라우저 개발자도구(F12)에서 디바이스 모드로 모바일 화면 테스트 가능

4. **그리드 디버깅**:
    - 크롬 개발자도구 → Elements → Computed → Grid 섹션
    - 그리드 라인을 시각적으로 확인 가능

---

## 📞 문제 해결

### 레이아웃이 깨졌을 때

1. `.dashboard-main`의 `grid-template-columns` 확인
2. 각 카드의 `grid-column`과 `grid-row` 값 확인
3. 미디어 쿼리가 적용되었는지 확인

### 색상이 안 바뀔 때

1. `:root`의 CSS 변수가 올바른지 확인
2. `var(--변수명)` 형식으로 사용했는지 확인
3. 브라우저 캐시 강제 새로고침

### 카드가 안 보일 때

1. `display` 속성 확인
2. `opacity`, `visibility` 확인
3. `z-index` 겹침 문제 확인
