"""
Google Custom Search API를 사용하여 웹 검색을 수행하는 모듈
"""

import os
import requests
from dotenv import load_dotenv

# 환경변수 로드
load_dotenv()

GOOGLE_API_KEY = os.getenv("GOOGLE_API_KEY")
GOOGLE_CSE_ID = os.getenv("GOOGLE_CSE_ID")

def search_web(query: str, num_results: int = 3, user_location: str = "") -> str:
    """
    Google Custom Search API를 사용하여 웹 검색을 수행하고 결과를 문자열로 반환합니다.
    
    Args:
        query (str): 검색어
        num_results (int): 가져올 결과 개수 (기본값: 3)
        user_location (str): 사용자 위치 (도시명, 예: '서울')
        
    Returns:
        str: 검색 결과 요약 텍스트
    """
    if not GOOGLE_API_KEY or not GOOGLE_CSE_ID:
        return "⚠️ Google 검색 API 키 또는 검색 엔진 ID가 설정되지 않았습니다."

    # 위치 정보가 있고, 쿼리에 위치가 없으면 위치 추가
    search_query = query
    if user_location and user_location not in query:
        search_query = f"{user_location} {query}"
        print(f"🔍 위치 기반 검색: {search_query}")

    url = "https://www.googleapis.com/customsearch/v1"
    params = {
        "key": GOOGLE_API_KEY,
        "cx": GOOGLE_CSE_ID,
        "q": search_query,
        "num": num_results,
        "hl": "ko"  # 한국어 검색
    }

    try:
        response = requests.get(url, params=params, timeout=5)
        response.raise_for_status()
        data = response.json()
        
        if "items" not in data:
            return "검색 결과가 없습니다."
            
        results = []
        for i, item in enumerate(data["items"]):
            title = item.get("title", "제목 없음")
            link = item.get("link", "")
            snippet = item.get("snippet", "내용 없음")
            
            results.append(f"{i+1}. 제목: {title}\n   링크: {link}\n   내용: {snippet}")
            
        return "\n\n".join(results)
        
    except Exception as e:
        print(f"⚠️ 검색 오류: {e}")
        return f"웹 검색 중 오류가 발생했습니다: {str(e)}"

# 테스트 코드
if __name__ == "__main__":
    print(search_web("오늘 서울 날씨"))
