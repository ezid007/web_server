#!/usr/bin/env python3
"""
HTML 파일의 남은 외부 링크를 로컬 경로로 변환
"""
import re


def fix_remaining_links(file_path):
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()

    # content 속성의 이미지 URL
    content = re.sub(
        r'content="https://reinostudio\.com\.br/wp-content/uploads/[^"]+/([^"]+)"',
        r'content="{{ url_for(\'static\', path=\'/images/\1\') }}"',
        content,
    )

    # CSS 파일
    content = re.sub(
        r'href="https://reinostudio\.com\.br/wp-content/themes/reino/assets/lib/SwiperJs/swiper-bundle\.min\.css"',
        r'href="{{ url_for(\'static\', path=\'/lib/SwiperJs/swiper-bundle.min.css\') }}"',
        content,
    )

    content = re.sub(
        r'href="https://reinostudio\.com\.br/wp-content/themes/reino/assets/css/style\.css"',
        r'href="{{ url_for(\'static\', path=\'/css/style.css\') }}"',
        content,
    )

    # JS 파일 (wpo-minify)
    content = re.sub(
        r'src="https://reinostudio\.com\.br/wp-content/cache/wpo-minify/[^"]+/assets/wpo-minify-footer-71110d17\.min\.js"',
        r'src="{{ url_for(\'static\', path=\'/js/wpo-minify-footer-71110d17.min.js\') }}"',
        content,
    )

    content = re.sub(
        r'src="https://reinostudio\.com\.br/wp-content/cache/wpo-minify/[^"]+/assets/wpo-minify-footer-68e80742\.min\.js"',
        r'src="{{ url_for(\'static\', path=\'/js/wpo-minify-footer-68e80742.min.js\') }}"',
        content,
    )

    # WordPress JSON API root
    content = re.sub(
        r'"root": "https:\\\/\\\/reinostudio\.com\.br\\\/wp-json\\\/"',
        r'"root": "{{ request.base_url }}api/"',
        content,
    )

    with open(file_path, "w", encoding="utf-8") as f:
        f.write(content)

    print(f"✓ {file_path} 수정 완료")


if __name__ == "__main__":
    files = [
        "/home/tuf/web_server/templates/works.html",
        "/home/tuf/web_server/templates/contact.html",
    ]

    for file_path in files:
        fix_remaining_links(file_path)

    print("\n모든 링크 수정 완료!")
