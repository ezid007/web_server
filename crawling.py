import time
from selenium import webdriver
from selenium.webdriver.chrome.service import Service
from webdriver_manager.chrome import ChromeDriverManager
from bs4 import BeautifulSoup


# 1. 크롬 브라우저 옵션 설정 (화면 없이 실행하려면 headless 옵션 추가)
options = webdriver.ChromeOptions()
# options.add_argument("headless") # 화면 안 띄우고 백그라운드에서 실행하려면 주석 해제

# 2. 브라우저 실행
print("🚀 브라우저를 실행합니다...")
driver = webdriver.Chrome(
    service=Service(ChromeDriverManager().install()), options=options
)

# 3. 목표 사이트 접속
url = "https://reinostudio.com.br/"
print(f"🌐 {url} 에 접속 중...")
driver.get(url)

# 4. 페이지 로딩 대기 (자바스크립트가 다 실행될 때까지 기다림)
time.sleep(5)  # 5초 대기 (네트워크 상황에 따라 조절)

# 5. HTML 가져오기
html = driver.page_source
soup = BeautifulSoup(html, "html.parser")

# 6. 파일로 저장하기
save_path = "templates/index_v2.html"
with open(save_path, "w", encoding="utf-8") as f:
    f.write(soup.prettify())  # 보기 좋게 정렬해서 저장

print(f"✅ 크롤링 완료! '{save_path}'에 저장되었습니다.")

# 7. 브라우저 종료
driver.quit()
