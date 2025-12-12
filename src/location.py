"""
IP 기반 위치 감지 모듈
ip-api.com API를 사용하여 클라이언트 IP 주소에서 위치 정보를 추출합니다.
main.py와 chatbot.py에서 공통으로 사용됩니다.
"""

import httpx
from functools import lru_cache

# 영어 → 한글 도시명 매핑
CITY_KOREAN = {
    "Seoul": "서울", "Busan": "부산", "Incheon": "인천", "Daegu": "대구",
    "Daejeon": "대전", "Gwangju": "광주", "Ulsan": "울산", "Sejong": "세종",
    "Suwon": "수원", "Suwon-si": "수원", "Seongnam-si": "성남", "Seongnam": "성남",
    "Yongin-si": "용인", "Yongin": "용인", "Goyang-si": "고양", "Bucheon-si": "부천",
    "Ansan-si": "안산", "Anyang-si": "안양", "Cheongju-si": "청주", "Jeonju": "전주",
    "Cheonan": "천안", "Gimhae-si": "김해", "Changwon-si": "창원", "Pohang-si": "포항",
    "Jeju City": "제주", "Jeju-si": "제주", "Gyeonggi-do": "경기도",
}

# 기본값: 서울
DEFAULT_LOCATION = {
    "city": "서울",
    "lat": 37.5665,
    "lon": 126.9780,
    "region": "서울특별시"
}


async def get_location_from_ip_async(ip_address: str = None) -> dict:
    """
    IP 주소 기반 위치 정보 조회 (비동기)
    
    Args:
        ip_address (str, optional): 조회할 IP 주소. None이면 서버의 공인 IP 사용.
        
    Returns:
        dict: {"city": "서울", "lat": 37.5665, "lon": 126.9780, "region": "서울특별시"}
    """
    try:
        async with httpx.AsyncClient(timeout=5.0) as client:
            # IP가 없거나 로컬 IP면 자동 감지
            url = "http://ip-api.com/json/?lang=ko"
            response = await client.get(url)
            data = response.json()
            
            if data.get("status") == "success":
                city_en = data.get("city", "Seoul")
                city_kr = CITY_KOREAN.get(city_en, city_en)
                location = {
                    "city": city_kr,
                    "lat": data.get("lat", 37.5665),
                    "lon": data.get("lon", 126.9780),
                    "region": data.get("regionName", ""),
                }
                print(f"📍 위치 감지: {city_kr} (IP: {data.get('query', 'auto')})")
                return location
                
    except Exception as e:
        print(f"⚠️ IP 위치 조회 실패: {e}")
    
    return DEFAULT_LOCATION.copy()


def get_location_from_ip_sync(ip_address: str = None) -> str:
    """
    IP 주소로 도시명만 반환 (동기 - LRU 캐시 적용)
    챗봇에서 사용합니다.
    
    Args:
        ip_address (str, optional): 조회할 IP 주소
        
    Returns:
        str: 도시명 (예: '서울', '성남')
    """
    return _get_city_cached(ip_address or "auto")


@lru_cache(maxsize=100)
def _get_city_cached(ip_address: str) -> str:
    """캐시된 도시명 조회 (동기)"""
    import requests
    
    try:
        response = requests.get("http://ip-api.com/json/?lang=ko", timeout=3)
        data = response.json()
        
        if data.get("status") == "success":
            city_en = data.get("city", "Seoul")
            city_kr = CITY_KOREAN.get(city_en, city_en)
            print(f"📍 위치 감지: {city_kr} (IP: {data.get('query', 'auto')})")
            return city_kr
            
    except Exception as e:
        print(f"⚠️ IP 위치 조회 실패: {e}")
    
    return "서울"


def needs_location_context(query: str) -> bool:
    """
    쿼리가 위치 정보가 필요한지 판단합니다.
    
    Args:
        query (str): 사용자 질문
        
    Returns:
        bool: 위치 정보가 필요하면 True
    """
    # 위치 관련 키워드
    location_keywords = [
        "날씨", "기온", "온도", "비", "눈", "미세먼지", "대기질",
        "식당", "음식점", "카페", "맛집", "병원", "약국",
        "근처", "주변", "가까운", "여기", "이곳",
    ]
    
    # 이미 위치가 명시되어 있는지 확인
    location_specified = any(loc in query for loc in [
        "서울", "부산", "대구", "인천", "광주", "대전", "울산", "세종",
        "경기", "강원", "충북", "충남", "전북", "전남", "경북", "경남", "제주"
    ])
    
    return any(kw in query for kw in location_keywords) and not location_specified


# 테스트 코드
if __name__ == "__main__":
    import asyncio
    print("Async:", asyncio.run(get_location_from_ip_async()))
    print("Sync:", get_location_from_ip_sync())
