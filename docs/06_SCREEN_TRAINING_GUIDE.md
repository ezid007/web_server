# 🚀 SSH 종료해도 학습 유지하는 방법 (screen 사용)

SSH로 원격 서버에 접속해서 학습을 시작한 후, SSH 연결을 끊어도 학습이 계속 진행되도록 하는 방법입니다.

---

## 📌 Step 1: 현재 학습 중지

지금 학습이 돌아가는 터미널에서:
```
Ctrl + C
```
> 학습이 중단되고 프롬프트가 돌아옵니다.

---

## 📌 Step 2: screen 설치 확인

```bash
screen --version
```

설치 안 되어 있으면:
```bash
sudo apt install screen -y
```

---

## 📌 Step 3: screen 세션 만들기

```bash
screen -S training
```
> 화면이 깜빡이고 새 세션에 들어갑니다.

---

## 📌 Step 4: 가상환경 활성화 & 학습 시작

```bash
cd /home/hak/web_server
source .venv/bin/activate
python auto_labeling/scripts/train.py
```
> 학습이 시작됩니다. 로그가 쭉 나오는 것 확인!

---

## 📌 Step 5: 세션 분리 (중요!)

학습이 돌아가는 상태에서 키보드 입력:
```
Ctrl + A   (먼저 누르고)
D          (그 다음 누름)
```
> `[detached from xxxx.training]` 메시지가 뜨면 성공!

---

## 📌 Step 6: 퇴근하세요! 🏠

이제 SSH 연결을 끊어도 됩니다:
```bash
exit
```
노트북 덮고 집에 가세요!

---

## 📌 Step 7: 집에서 확인하기

집에서 SSH 다시 접속 후:
```bash
screen -r training
```
> 학습 진행 상황이 그대로 보입니다!

---

## ⌨️ screen 명령어 요약

| 상황 | 명령어 |
|------|--------|
| 세션 목록 보기 | `screen -ls` |
| 세션 재접속 | `screen -r training` |
| 세션 분리 | `Ctrl+A`, `D` |
| 세션 완전 종료 | `Ctrl+A`, `K`, `y` |

---

## 💡 왜 이게 가능한가?

```
🖥️ 학습 서버 (회사)
├── screen 세션 ← 서버 메모리에 존재
│   └── python train.py 실행 중 ← 서버 GPU에서 돌아감
│
└── SSH 연결 끊김? 상관없음!

💻 노트북 (가져감)
└── SSH 종료됨 → 아무 영향 없음
```

- **screen 세션**은 서버에 존재
- **학습 프로세스**는 서버 GPU에서 실행
- **노트북**은 그냥 "보는 창"일 뿐

---

## 🔧 문제 해결

### "There is no screen to be resumed"
```bash
screen -ls  # 세션 목록 확인
```

### 세션이 "Attached" 상태일 때
```bash
screen -d -r training  # 강제로 재접속
```

### 여러 세션이 있을 때
```bash
screen -r [세션ID]  # 특정 세션 접속
```
