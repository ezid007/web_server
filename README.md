# Self-contained FastAPI Web Server

이 프로젝트는 모든 정적 파일(이미지, 폰트, CSS, JS 등)이 `/static/` 폴더에 포함되어 있어, `web_server` 폴더만 복사하면 바로 배포 가능합니다.

## 로컬 실행 방법 (Windows)

1. Python 3.10+ 설치
2. Poetry 설치 (PowerShell)
    ```powershell
    (Invoke-WebRequest -Uri https://install.python-poetry.org -UseBasicParsing).Content | python -
    ```
    _설치 후 `poetry` 명령어가 실행되지 않으면 환경 변수(PATH)에 추가가 필요할 수 있습니다._
3. 의존성 설치
    ```powershell
    poetry install
    ```
4. 서버 실행
    ```powershell
    poetry run uvicorn main:app --host 127.0.0.1 --port 8000 --reload
    ```

## AWS EC2 배포 예시

1. EC2 인스턴스에 `web_server` 폴더 업로드
2. Python 및 필요한 패키지 설치
3. 아래와 같이 systemd 서비스 파일 생성

### systemd 서비스 예시

`/etc/systemd/system/web_server.service`

```ini
[Unit]
Description=FastAPI Web Server
After=network.target

[Service]
User=ec2-user
WorkingDirectory=/home/ec2-user/web_server
ExecStart=/home/ec2-user/web_server/.venv/bin/uvicorn main:app --host 127.0.0.1 --port 8000
Restart=always

[Install]
WantedBy=multi-user.target
```

서비스 등록 및 시작:

```bash
sudo systemctl daemon-reload
sudo systemctl enable web_server
sudo systemctl start web_server
```

### Nginx Reverse Proxy 예시

`/etc/nginx/conf.d/web_server.conf`

```nginx
server {
    listen 80;
    server_name example.com;

    location / {
        proxy_pass http://127.0.0.1:8000;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
    }

    location /static/ {
        alias /home/ec2-user/web_server/static/;
        access_log off;
        expires 30d;
    }
}
```

Nginx 재시작:

```bash
sudo systemctl restart nginx
```

## 참고 사항

-   모든 외부 의존성(이미지, 폰트, JS, CSS 등)이 로컬에 포함되어 있습니다.
-   템플릿 내 모든 외부 링크/스크립트/이미지는 제거 또는 로컬로 대체되었습니다.
-   추가 보안 설정 및 HTTPS 적용은 별도 진행 바랍니다.
