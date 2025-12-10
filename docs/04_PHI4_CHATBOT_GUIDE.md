# PHi-4 챗봇 사용 설명서

## 📋 목차
1. [개요](#개요)
2. [폴더 구조](#폴더-구조)
3. [환경 설정](#환경-설정)
4. [학습 데이터 준비](#학습-데이터-준비)
5. [데이터셋 생성](#데이터셋-생성)
6. [모델 학습](#모델-학습)
7. [학습된 모델 사용](#학습된-모델-사용)
8. [웹 통합](#웹-통합)
9. [문제 해결](#문제-해결)

---

## 개요

PHi-4는 **Phi-4 논문**의 핵심 아이디어를 구현한 한국어 추론 AI 챗봇입니다.

### 핵심 원리
- **Synthetic Data Generation**: 원시 텍스트를 고품질 추론 문제로 변환
- **Chain-of-Thought (CoT)**: 단계별 논리적 사고 훈련
- **LoRA Fine-tuning**: 적은 메모리로 대형 모델 학습

### 시스템 요구사항
| 항목 | 최소 사양 | 권장 사양 |
|------|----------|----------|
| GPU | 8GB VRAM | 12GB+ VRAM |
| RAM | 16GB | 32GB+ |
| 저장공간 | 20GB | 50GB+ |

---

## 폴더 구조

```
Phi-4/
├── data/
│   ├── seeds/              # 원시 학습 데이터 (텍스트 파일)
│   │   ├── medical/        # 의약품 정보 텍스트
│   │   └── example.txt     # 시드 텍스트 예시
│   └── synthetic/
│       └── train.jsonl     # 생성된 학습 데이터셋
│
├── src/
│   ├── pipeline/           # 데이터 생성 파이프라인
│   │   ├── generate_dataset.py   # 데이터셋 생성 메인 스크립트
│   │   ├── generator.py          # Ollama 기반 합성 데이터 생성기
│   │   └── schemas.py            # Pydantic 데이터 스키마
│   │
│   ├── train/              # 모델 학습
│   │   └── train.py        # SFT 학습 스크립트
│   │
│   ├── test/               # 테스트
│   │   └── inference.py    # 추론 테스트
│   │
│   └── utils/              # 유틸리티
│       └── inspect_data.py # 데이터 검사 도구
│
└── models/                 # 학습된 모델 저장 위치 (git 제외)
    └── phi-4-3b-reasoning/ # LoRA 어댑터
```

---

## 환경 설정

### 1. Ollama 설치 (데이터 생성용)

```bash
# Ollama 설치
curl -fsSL https://ollama.ai/install.sh | sh

# 데이터 생성용 모델 다운로드
ollama pull qwen2.5:7b
```

### 2. Python 의존성

프로젝트 루트(`web_server/`)에서:

```bash
poetry install
```

주요 의존성:
- `torch>=2.0.0` - PyTorch
- `transformers>=4.40.0` - Hugging Face 트랜스포머
- `peft>=0.10.0` - LoRA 학습
- `bitsandbytes>=0.43.0` - 4비트 양자화
- `accelerate>=0.28.0` - GPU 최적화
- `langchain-ollama` - Ollama 연동

---

## 학습 데이터 준비

### 시드 텍스트 형식

`data/seeds/` 폴더에 `.txt` 파일로 원시 텍스트를 저장합니다.

**예시: `data/seeds/medical/aspirin.txt`**

```text
아스피린(Aspirin)은 아세틸살리실산(acetylsalicylic acid)의 상품명으로,
해열, 진통, 항염증 효과가 있는 비스테로이드 항염증제(NSAID)입니다.

작용 원리:
아스피린은 사이클로옥시게나제(COX) 효소를 비가역적으로 억제하여
프로스타글란딘의 합성을 차단합니다.

적응증:
- 두통, 치통, 근육통
- 발열 감소
- 혈전 예방 (저용량)

주의사항:
- 위장관 출혈 위험
- 12세 미만 어린이 사용 금지 (레이 증후군)
- 임신 3기 사용 금지
```

### 좋은 시드 텍스트 작성 가이드

| ✅ 권장 | ❌ 피해야 할 것 |
|--------|---------------|
| 구체적인 사실 포함 | 모호한 일반론 |
| 논리적 인과관계 | 단순 나열 |
| 전문 용어 + 설명 | 약어만 사용 |
| 500~2000자 | 너무 짧거나 긴 텍스트 |

---

## 데이터셋 생성

### 1. Ollama 서버 실행 확인

```bash
ollama serve  # 별도 터미널에서 실행
```

### 2. 합성 데이터 생성

```bash
cd Phi-4
python -m src.pipeline.generate_dataset
```

**출력 예시:**
```
📂 Found 15 seed files in 'data/seeds'
🚀 Starting batch generation... (This may take time)
[1/15] Processing: aspirin.txt... ✅ Done
[2/15] Processing: ibuprofen.txt... ✅ Done
...
🎉 Generation Complete!
📊 Success Rate: 14/15
💾 Saved to: data/synthetic/train.jsonl
```

### 3. 생성된 데이터 확인

```bash
python -m src.utils.inspect_data
```

**생성되는 JSONL 형식:**
```json
{
  "question": "아스피린이 혈전을 예방하는 원리는 무엇인가요?",
  "reasoning_steps": [
    "아스피린은 COX 효소를 억제합니다.",
    "COX 억제로 트롬복산 A2 생성이 감소합니다.",
    "트롬복산 A2는 혈소판 응집을 촉진하는 물질입니다.",
    "따라서 혈소판 응집이 억제되어 혈전 형성이 줄어듭니다."
  ],
  "answer": "아스피린은 COX 효소를 비가역적으로 억제하여 혈소판의 트롬복산 A2 생성을 차단하고, 이를 통해 혈소판 응집을 억제하여 혈전 형성을 예방합니다."
}
```

---

## 모델 학습

### 1. 학습 실행

```bash
cd Phi-4
python -m src.train.train
```

### 2. 주요 설정 (train.py)

```python
# 기본 모델 (3B 파라미터)
model_name = "Qwen/Qwen2.5-3B-Instruct"

# 메모리 최적화 설정
per_device_train_batch_size = 1   # 배치 크기 1 (안전)
gradient_accumulation_steps = 8   # 누적 배치 8
max_length = 512                  # 최대 토큰 길이

# LoRA 설정
r = 16            # 어댑터 랭크
lora_alpha = 32   # 학습 반영률
```

### 3. 학습 시간 예상

| 데이터 수 | GPU | 예상 시간 |
|----------|-----|----------|
| 100개 | RTX 3070 8GB | 1-2시간 |
| 500개 | RTX 3070 8GB | 5-8시간 |
| 1000개 | RTX 4090 24GB | 3-4시간 |

### 4. 학습 결과

학습 완료 시 `models/phi-4-3b-reasoning/` 폴더에 저장됩니다:
- `adapter_config.json` - LoRA 설정
- `adapter_model.safetensors` - 학습된 가중치

---

## 학습된 모델 사용

### 추론 테스트

```bash
cd Phi-4
python -m src.test.inference
```

### Python 코드에서 사용

```python
import torch
from transformers import AutoTokenizer, AutoModelForCausalLM, BitsAndBytesConfig
from peft import PeftModel

# 4비트 양자화 설정
bnb_config = BitsAndBytesConfig(
    load_in_4bit=True,
    bnb_4bit_quant_type="nf4",
    bnb_4bit_compute_dtype=torch.float16,
)

# 기본 모델 로드
base_model = AutoModelForCausalLM.from_pretrained(
    "Qwen/Qwen2.5-3B-Instruct",
    quantization_config=bnb_config,
    device_map="auto"
)
tokenizer = AutoTokenizer.from_pretrained("Qwen/Qwen2.5-3B-Instruct")

# 학습된 어댑터 합체
model = PeftModel.from_pretrained(base_model, "models/phi-4-3b-reasoning")

# 추론
prompt = "<|im_start|>user\n아스피린의 작용 원리를 설명해주세요.<|im_end|>\n<|im_start|>assistant\n"
inputs = tokenizer(prompt, return_tensors="pt").to("cuda")

with torch.no_grad():
    outputs = model.generate(**inputs, max_new_tokens=512)

print(tokenizer.decode(outputs[0], skip_special_tokens=True))
```

---

## 웹 통합

### 현재 구조

학습된 모델은 웹 서버와 다음과 같이 통합되어 있습니다:

```
web_server/
├── models/phi-4-reasoning/    # 학습된 어댑터 (from Phi-4/models/)
├── src/chatbot.py             # 챗봇 API 모듈
└── main.py                    # FastAPI 서버 (챗봇 라우터 포함)
```

### 환경 변수 설정

`.env` 파일에 추가:
```
PHI4_ADAPTER_PATH=phi-4-reasoning
```

### API 엔드포인트

| 메서드 | 경로 | 설명 |
|--------|------|------|
| POST | `/api/chat` | 챗봇 대화 |
| GET | `/api/chat/status` | 모델 로드 상태 확인 |

---

## 문제 해결

### OOM (Out of Memory) 에러

```bash
torch.cuda.OutOfMemoryError: CUDA out of memory
```

**해결책:**
1. `train.py`에서 `per_device_train_batch_size=1` 확인
2. `tokenizer.model_max_length = 512` (더 짧게 설정)
3. 다른 GPU 프로세스 종료: `nvidia-smi` 확인

### Ollama 연결 실패

```bash
ConnectionError: Failed to connect to Ollama
```

**해결책:**
```bash
# Ollama 서버 상태 확인
curl http://localhost:11434/api/tags

# 서버 재시작
sudo systemctl restart ollama
```

### 챗봇 응답 품질 저하

모델이 질문에 직접 답하지 않는 경우:

1. **시드 데이터 품질 개선**
   - 더 구체적이고 논리적인 텍스트 사용
   
2. **프롬프트 형식 확인**
   - ChatML 형식 (`<|im_start|>`, `<|im_end|>`) 사용

3. **학습 데이터 양 증가**
   - 최소 100개 이상의 고품질 시드 권장

---

## 참고 자료

- [Phi-4 Technical Report (Microsoft)](https://arxiv.org/abs/2412.08905)
- [LoRA Paper](https://arxiv.org/abs/2106.09685)
- [QLoRA Paper](https://arxiv.org/abs/2305.14314)
- [Hugging Face PEFT Documentation](https://huggingface.co/docs/peft)
