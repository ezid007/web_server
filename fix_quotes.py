#!/usr/bin/env python3
"""이스케이프된 따옴표 제거"""


def fix_escaped_quotes(file_path):
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()

    # \' 를 ' 로 변경
    content = content.replace(r"\'", "'")

    with open(file_path, "w", encoding="utf-8") as f:
        f.write(content)

    print(f"✓ {file_path} 수정 완료")


if __name__ == "__main__":
    files = [
        "/home/tuf/web_server/templates/works.html",
        "/home/tuf/web_server/templates/contact.html",
    ]

    for file_path in files:
        fix_escaped_quotes(file_path)

    print("\n이스케이프 문자 제거 완료!")
