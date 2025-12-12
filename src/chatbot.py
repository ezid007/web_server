"""
PHi-4 챗봇 모듈
한국어 추론 AI 모델을 로드하고 채팅 API를 제공합니다.
"""

import os
import torch
from pathlib import Path
from fastapi import APIRouter
from fastapi.responses import JSONResponse

# 환경 변수에서 모델 경로 로드
_base_dir = Path(__file__).parent.parent
_models_dir = _base_dir / "models"

def _resolve_model_path(model_name: str, default: str) -> Path:
    """모델 경로 해석: 파일명만 있으면 models/ 폴더에서 찾음"""
    path = Path(model_name) if model_name else Path(default)
    if path.is_absolute():
        return path
    if path.parent == Path("."):
        return _models_dir / path
    return _base_dir / path

PHI4_ADAPTER_PATH = _resolve_model_path(
    os.getenv("PHI4_ADAPTER_PATH", ""), "phi-4-reasoning"
)

# 전역 변수로 모델과 토크나이저 선언
chatbot_model = None
chatbot_tokenizer = None
chatbot_loaded = False

# 라우터 생성
router = APIRouter(prefix="/api", tags=["chatbot"])


async def load_chatbot_model():
    """PHi-4 모델 로드 (서버 시작 시 호출)"""
    global chatbot_model, chatbot_tokenizer, chatbot_loaded
    
    if chatbot_loaded:
        return
    
    # 모델 경로 확인
    if not PHI4_ADAPTER_PATH.exists():
        print(f"⚠️ PHi-4 모델을 찾을 수 없습니다: {PHI4_ADAPTER_PATH}")
        return
    
    try:
        from transformers import AutoTokenizer, AutoModelForCausalLM, BitsAndBytesConfig
        from peft import PeftModel
        
        print("⏳ PHi-4 모델을 로딩 중입니다... (약 1~2분 소요)")
        
        base_model_name = "Qwen/Qwen2.5-3B-Instruct"
        
        # 8GB VRAM을 위한 4비트 양자화 설정
        bnb_config = BitsAndBytesConfig(
            load_in_4bit=True,
            bnb_4bit_quant_type="nf4",
            bnb_4bit_use_double_quant=True,
            bnb_4bit_compute_dtype=torch.float16,
        )
        
        # 원본 모델 로드
        base_model = AutoModelForCausalLM.from_pretrained(
            base_model_name, quantization_config=bnb_config, device_map="auto"
        )
        chatbot_tokenizer = AutoTokenizer.from_pretrained(base_model_name)
        
        # 학습된 어댑터 합체
        chatbot_model = PeftModel.from_pretrained(base_model, str(PHI4_ADAPTER_PATH))
        chatbot_loaded = True
        print("✅ PHi-4 모델 로딩 완료! 한국어 추론 준비 끝.")
        
    except Exception as e:
        print(f"⚠️ PHi-4 모델 로드 실패: {e}")


async def unload_chatbot_model():
    """PHi-4 모델 언로드 (서버 종료 시 호출)"""
    global chatbot_model, chatbot_tokenizer, chatbot_loaded
    
    if chatbot_model:
        del chatbot_model
        chatbot_model = None
    if chatbot_tokenizer:
        del chatbot_tokenizer
        chatbot_tokenizer = None
    
    chatbot_loaded = False
    torch.cuda.empty_cache()


from pydantic import BaseModel
from typing import List, Optional

class ChatMessage(BaseModel):
    sender: str
    text: str

class ChatRequest(BaseModel):
    prompt: str
    history: Optional[List[ChatMessage]] = None

@router.post("/chat")
async def chat(request: ChatRequest):
    """사용자의 질문을 받아 모델이 추론하고 답변을 반환합니다."""
    global chatbot_model, chatbot_tokenizer, chatbot_loaded
    
    if not chatbot_loaded:
        return JSONResponse(
            content={"response": "⚠️ 챗봇 모델이 로드되지 않았습니다."},
            status_code=503
        )
    
    try:
        # 시스템 프롬프트
        system_prompt = (
            "<|im_start|>system\n"
            "당신은 의약학 지식을 갖춘 유능한 한국어 AI 비서입니다. "
            "사용자의 질문에 대해 정확하고 상세하게 한국어로 답변하세요. "
            "이전 대화 내용을 참고하여 맥락에 맞게 답변하세요.<|im_end|>\n"
        )
        
        # 대화 내역을 프롬프트에 추가
        history_prompt = ""
        if request.history:
            for msg in request.history:
                if msg.sender == "user":
                    history_prompt += f"<|im_start|>user\n{msg.text}<|im_end|>\n"
                elif msg.sender == "bot":
                    history_prompt += f"<|im_start|>assistant\n{msg.text}<|im_end|>\n"
        
        # 현재 사용자 질문
        user_prompt = f"<|im_start|>user\n{request.prompt}<|im_end|>\n"
        assistant_start = "<|im_start|>assistant\n"
        
        full_prompt = system_prompt + history_prompt + user_prompt + assistant_start
        
        inputs = chatbot_tokenizer(full_prompt, return_tensors="pt").to("cuda")
        
        # 추론 생성
        with torch.no_grad():
            outputs = chatbot_model.generate(
                **inputs,
                max_new_tokens=1024,
                temperature=0.7,
                top_p=0.9,
                do_sample=True,
                eos_token_id=chatbot_tokenizer.eos_token_id,
                pad_token_id=chatbot_tokenizer.eos_token_id,
            )
        
        # 결과 디코딩
        generated_text = chatbot_tokenizer.decode(outputs[0], skip_special_tokens=True)
        
        # 어시스턴트 답변만 추출
        # 1. 사용자 질문 이후 부분 추출
        if request.prompt in generated_text:
            answer_start = generated_text.find(request.prompt) + len(request.prompt)
            final_answer = generated_text[answer_start:].strip()
        else:
            final_answer = generated_text
        
        # 2. "assistant\n" 접두어 제거
        if final_answer.startswith("assistant"):
            final_answer = final_answer[len("assistant"):].strip()
        
        return JSONResponse(content={"response": final_answer})
        
    except Exception as e:
        print(f"⚠️ 챗봇 추론 오류: {e}")
        return JSONResponse(
            content={"response": f"⚠️ 오류가 발생했습니다: {str(e)}"},
            status_code=500
        )


@router.get("/chat/status")
async def chat_status():
    """챗봇 모델 로드 상태 확인"""
    return JSONResponse(content={
        "loaded": chatbot_loaded,
        "model_path": str(PHI4_ADAPTER_PATH),
        "model_exists": PHI4_ADAPTER_PATH.exists()
    })
