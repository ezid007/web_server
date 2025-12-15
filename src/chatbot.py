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
from src.location import get_location_from_ip_sync, get_location_from_ip_async, needs_location_context

# 날씨 API용 import
import httpx
from datetime import datetime, timedelta

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
    "뉴스", "최신", "주가", "환율",
    "검색", "찾아", "알려줘", "몇시", "누가", "언제",
    "실시간", "속보", "경기", "결과", "스코어", "순위"
]

# 날씨 API가 필요한 키워드 목록
WEATHER_KEYWORDS = ["날씨", "기온", "온도", "습도", "비", "눈", "맑음", "흐림", "몇도"]


def needs_weather_api(query: str) -> bool:
    """질문이 날씨 API를 필요로 하는지 판단합니다."""
    return any(keyword in query for keyword in WEATHER_KEYWORDS)


def needs_web_search(query: str) -> bool:
    """질문이 웹 검색을 필요로 하는지 판단합니다."""
    # 날씨 관련 질문은 웹 검색 대신 날씨 API 사용
    if needs_weather_api(query):
        return False
    return any(keyword in query for keyword in SEARCH_KEYWORDS)


# 기상청 API 키
WEATHER_API_KEY = os.getenv("WEATHER_API_KEY")

# 하늘 상태 코드
SKY_STATUS = {
    "1": "맑음", "3": "구름많음", "4": "흐림"
}

# 강수 형태 코드
PTY_STATUS = {
    "0": "없음", "1": "비", "2": "비/눈", "3": "눈", 
    "4": "소나기", "5": "빗방울", "6": "빗방울눈날림", "7": "눈날림"
}


def _latlon_to_grid(lat: float, lon: float) -> dict:
    """위도/경도를 기상청 격자 좌표로 변환"""
    import math
    
    RE = 6371.00877
    GRID = 5.0
    SLAT1 = 30.0
    SLAT2 = 60.0
    OLON = 126.0
    OLAT = 38.0
    XO = 43
    YO = 136
    
    DEGRAD = math.pi / 180.0
    
    re = RE / GRID
    slat1 = SLAT1 * DEGRAD
    slat2 = SLAT2 * DEGRAD
    olon = OLON * DEGRAD
    olat = OLAT * DEGRAD
    
    sn = math.tan(math.pi * 0.25 + slat2 * 0.5) / math.tan(math.pi * 0.25 + slat1 * 0.5)
    sn = math.log(math.cos(slat1) / math.cos(slat2)) / math.log(sn)
    sf = math.tan(math.pi * 0.25 + slat1 * 0.5)
    sf = math.pow(sf, sn) * math.cos(slat1) / sn
    ro = math.tan(math.pi * 0.25 + olat * 0.5)
    ro = re * sf / math.pow(ro, sn)
    
    ra = math.tan(math.pi * 0.25 + lat * DEGRAD * 0.5)
    ra = re * sf / math.pow(ra, sn)
    theta = lon * DEGRAD - olon
    if theta > math.pi:
        theta -= 2.0 * math.pi
    if theta < -math.pi:
        theta += 2.0 * math.pi
    theta *= sn
    
    nx = int(ra * math.sin(theta) + XO + 0.5)
    ny = int(ro - ra * math.cos(theta) + YO + 0.5)
    
    return {"nx": nx, "ny": ny}


async def get_weather_info(client_ip: str = None) -> str:
    """기상청 API를 통해 현재 날씨 정보를 문자열로 반환합니다."""
    try:
        # IP 기반 위치 조회
        location_info = await get_location_from_ip_async(client_ip)
        location = location_info["city"]
        
        # 위경도 → 격자 좌표 변환
        grid = _latlon_to_grid(location_info["lat"], location_info["lon"])
        
        # 기상청 API 시간 계산
        now = datetime.now()
        base_time = now.strftime("%H") + "00"
        base_date = now.strftime("%Y%m%d")
        
        if now.minute < 40:
            prev_hour = now.hour - 1
            if prev_hour < 0:
                prev_hour = 23
                base_date = (now - timedelta(days=1)).strftime("%Y%m%d")
            base_time = f"{prev_hour:02d}00"
        
        url = "http://apis.data.go.kr/1360000/VilageFcstInfoService_2.0/getUltraSrtNcst"
        params = {
            "serviceKey": WEATHER_API_KEY,
            "numOfRows": "10",
            "pageNo": "1",
            "dataType": "JSON",
            "base_date": base_date,
            "base_time": base_time,
            "nx": grid["nx"],
            "ny": grid["ny"],
        }
        
        async with httpx.AsyncClient(timeout=10.0) as client:
            response = await client.get(url, params=params)
            data = response.json()
        
        weather_data = {
            "temperature": None,
            "pty": "0",
            "wind_speed": None,
            "humidity": None,
        }
        
        if "response" in data and "body" in data["response"]:
            items = data["response"]["body"].get("items", {}).get("item", [])
            for item in items:
                category = item.get("category")
                value = item.get("obsrValue")
                
                if category == "T1H":
                    weather_data["temperature"] = float(value)
                elif category == "PTY":
                    weather_data["pty"] = str(int(float(value)))
                elif category == "WSD":
                    weather_data["wind_speed"] = float(value)
                elif category == "REH":
                    weather_data["humidity"] = float(value)
        
        # 날씨 상태 결정
        pty = weather_data["pty"]
        if pty != "0" and pty in PTY_STATUS:
            sky_name = PTY_STATUS[pty]
        else:
            sky_name = "맑음"
        
        # 결과 문자열 생성
        temp = weather_data["temperature"] if weather_data["temperature"] else "측정 불가"
        humidity = weather_data["humidity"] if weather_data["humidity"] else "측정 불가"
        wind = weather_data["wind_speed"] if weather_data["wind_speed"] else "측정 불가"
        
        result = (
            f"[{location} 현재 날씨]\n"
            f"- 기온: {temp}°C\n"
            f"- 날씨: {sky_name}\n"
            f"- 습도: {humidity}%\n"
            f"- 풍속: {wind}m/s\n"
            f"- 측정 시간: {now.strftime('%Y-%m-%d %H:%M')}"
        )
        
        return result
        
    except Exception as e:
        print(f"⚠️ 날씨 API 오류: {e}")
        return f"날씨 정보를 가져오는 중 오류가 발생했습니다: {str(e)}"


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
        # 날씨 API 또는 웹 검색 수행
        search_context = ""
        searched_web = False
        used_weather_api = False
        user_location = ""
        
        # 날씨 관련 질문인 경우 기상청 API 사용
        if needs_weather_api(request.prompt):
            print(f"🌤️ 날씨 API 호출: {request.prompt}")
            weather_result = await get_weather_info(request.client_ip)
            
            if weather_result and "오류" not in weather_result:
                used_weather_api = True
                search_context = (
                    f"\n\n[날씨 정보]\n{weather_result}\n"
                    "위 날씨 정보를 바탕으로 사용자의 질문에 자연스럽게 답변하세요."
                )
                print(f"✅ 날씨 정보 획득")
        
        # 날씨 외 질문은 웹 검색 수행
        elif needs_web_search(request.prompt):
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
        
        # 시스템 프롬프트 (날씨 정보 또는 검색 결과 포함)
        if used_weather_api:
            system_prompt = (
                "<|im_start|>system\n"
                "당신은 친근한 한국어 AI 날씨 비서입니다.\n"
                "중요: 아래 [날씨 정보]를 바탕으로 답변하세요.\n"
                "규칙:\n"
                "1. 날씨 정보를 자연스러운 대화체로 전달하세요.\n"
                "2. 온도, 날씨 상태, 습도 정보를 포함해서 답변하세요.\n"
                "3. 필요시 옷차림이나 우산 등 간단한 조언을 덧붙여도 좋습니다.\n"
                "4. 이모지는 사용하지 마세요."
                f"{search_context}<|im_end|>\n"
            )
        elif searched_web:
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
            "searched_web": searched_web,
            "used_weather_api": used_weather_api
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
