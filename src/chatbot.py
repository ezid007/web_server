"""
PHi-4 챗봇 모듈
한국어 추론 AI 모델을 로드하고 채팅 API를 제공합니다.
웹 검색 기능을 통해 최신 정보를 제공할 수 있습니다.
"""

import os
import torch
from pathlib import Path
from fastapi import APIRouter
from fastapi.responses import JSONResponse

# 웹 검색 및 위치 감지 모듈 import
from src.web_search import search_web
from src.location import get_location_from_ip_sync, needs_location_context

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

# 웹 검색이 필요한 키워드 목록
SEARCH_KEYWORDS = [
    "날씨", "오늘", "뉴스", "최신", "현재", "지금", "주가", "환율",
    "검색", "찾아", "알려줘", "몇시", "몇도", "어디", "누가", "언제",
    "실시간", "속보", "경기", "결과", "스코어", "순위"
]


def needs_web_search(query: str) -> bool:
    """질문이 웹 검색을 필요로 하는지 판단합니다."""
    return any(keyword in query for keyword in SEARCH_KEYWORDS)


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
    client_ip: Optional[str] = None

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
        # 웹 검색 필요 여부 확인 및 검색 수행
        search_context = ""
        searched_web = False
        user_location = ""
        
        if needs_web_search(request.prompt):
            # 위치 정보가 필요한 경우 IP로 위치 감지
            if needs_location_context(request.prompt):
                user_location = get_location_from_ip_sync(request.client_ip)
            
            print(f"🔍 웹 검색 수행: {request.prompt}" + (f" (위치: {user_location})" if user_location else ""))
            search_results = search_web(request.prompt, user_location=user_location)
            
            if search_results and "오류" not in search_results:
                searched_web = True
                search_context = (
                    f"\n\n[웹 검색 결과]\n{search_results}\n"
                    "위 검색 결과를 참고하여 사용자의 질문에 정확하게 답변하세요."
                )
                print(f"✅ 검색 결과 획득")
        
        # 시스템 프롬프트 (검색 결과 포함)
        # Hallucination 방지를 위해 검색 결과 기반 답변 강제
        if searched_web:
            system_prompt = (
                "<|im_start|>system\n"
                "당신은 정확한 정보를 전달하는 한국어 AI 비서입니다.\n"
                "중요: 아래 [웹 검색 결과]에 있는 정보만 사용하여 답변하세요.\n"
                "규칙:\n"
                "1. 검색 결과에 없는 내용은 절대 만들어내지 마세요.\n"
                "2. 검색 결과를 쉽고 간단한 일상 언어로 요약해서 전달하세요.\n"
                "3. 확실하지 않은 정보는 '~로 보입니다', '~라고 합니다'처럼 표현하세요.\n"
                "4. URL, 출처, 이모지는 사용하지 마세요.\n"
                "5. 핵심 정보만 2-3문장으로 간결하게 답변하세요."
                f"{search_context}<|im_end|>\n"
            )
        else:
            system_prompt = (
                "<|im_start|>system\n"
                "당신은 친근하고 도움이 되는 한국어 AI 비서입니다.\n"
                "규칙:\n"
                "1. 쉽고 간단한 일상 언어로 답변하세요.\n"
                "2. 확실히 아는 정보만 답변하고, 모르면 솔직히 '잘 모르겠습니다'라고 하세요.\n"
                "3. 사실이 아닌 정보를 만들어내지 마세요.\n"
                "4. 이모지는 사용하지 마세요.<|im_end|>\n"
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
        
        # 어시스턴트 답변만 추출 (더 강력한 파싱)
        final_answer = generated_text
        
        # 1. 마지막 "assistant" 이후 부분만 추출
        if "assistant" in final_answer:
            final_answer = final_answer.split("assistant")[-1].strip()
        
        # 2. "user" 마커가 남아있으면 그 앞부분만 사용 (다음 대화 시작 제거)
        if "\nuser" in final_answer:
            final_answer = final_answer.split("\nuser")[0].strip()
        if final_answer.startswith("user"):
            # 첫 줄이 user로 시작하면 제거
            lines = final_answer.split("\n")
            final_answer = "\n".join(lines[1:]).strip()
        
        # 3. 사용자 질문이 반복되면 제거
        if request.prompt in final_answer:
            final_answer = final_answer.replace(request.prompt, "").strip()
        
        # 4. 🔍 이모지로 시작하는 줄 제거 (검색 알림과 중복 방지)
        lines = final_answer.split("\n")
        filtered_lines = [line for line in lines if not line.strip().startswith("🔍")]
        final_answer = "\n".join(filtered_lines).strip()
        
        # 5. URL 포함 문장 제거 및 정리
        import re
        # URL 제거
        final_answer = re.sub(r'https?://\S+', '', final_answer)
        # "~에 가면", "~에서" 같은 불완전 시작 제거
        final_answer = re.sub(r'^[\s]*에\s+(가면|보면|확인)', '', final_answer)
        # 여러 공백을 하나로
        final_answer = re.sub(r'\s+', ' ', final_answer).strip()
        # 빈 응답이면 기본 메시지
        if not final_answer or len(final_answer) < 10:
            final_answer = "죄송합니다, 해당 정보를 찾지 못했습니다. 다른 질문을 해주세요."
        
        return JSONResponse(content={
            "response": final_answer,
            "searched_web": searched_web
        })
        
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
