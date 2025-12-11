#!/usr/bin/env python3
"""
원격 서버에서 YOLO 모델을 동기화하는 스크립트

사용법:
    python scripts/sync_yolo_model.py

환경변수 (.env):
    REMOTE_MODEL_HOST: 원격 서버 IP
    REMOTE_MODEL_USER: SSH 사용자명
    REMOTE_MODEL_PASSWORD: SSH 비밀번호
    REMOTE_MODEL_PATH: 원격 모델 파일 경로
"""

import os
import sys
from pathlib import Path

# 프로젝트 루트 디렉토리를 sys.path에 추가
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from dotenv import load_dotenv

# .env 파일 로드
load_dotenv(project_root / ".env")

# paramiko 임포트 (SFTP 사용)
try:
    import paramiko
except ImportError:
    print("❌ paramiko가 설치되어 있지 않습니다.")
    print("   설치 명령어: pip install paramiko")
    sys.exit(1)


def get_env_or_exit(key: str) -> str:
    """환경변수를 가져오고, 없으면 에러 출력 후 종료"""
    value = os.getenv(key)
    if not value:
        print(f"❌ 환경변수 '{key}'가 .env 파일에 설정되어 있지 않습니다.")
        sys.exit(1)
    return value


def sync_yolo_model():
    """원격 서버에서 YOLO 모델을 다운로드"""
    
    # 환경변수에서 설정 읽기
    host = get_env_or_exit("REMOTE_MODEL_HOST")
    user = get_env_or_exit("REMOTE_MODEL_USER")
    password = get_env_or_exit("REMOTE_MODEL_PASSWORD")
    remote_path = get_env_or_exit("REMOTE_MODEL_PATH")
    
    # 로컬 저장 경로 설정
    model_name = os.getenv("YOLO_DASHBOARD_MODEL", "my_yolo.pt")
    local_dir = project_root / "models"
    local_path = local_dir / model_name
    
    # 로컬 디렉토리 생성
    local_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"📡 원격 서버에서 YOLO 모델 동기화 중...")
    print(f"   서버: {user}@{host}")
    print(f"   원격 경로: {remote_path}")
    print(f"   로컬 경로: {local_path}")
    print()
    
    try:
        # SSH 클라이언트 생성
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        
        # 서버 연결
        print("🔗 서버에 연결 중...")
        ssh.connect(host, username=user, password=password, timeout=10)
        print("✅ 서버 연결 성공!")
        
        # SFTP 세션 열기
        sftp = ssh.open_sftp()
        
        # ~ 경로 확장 (홈 디렉토리)
        if remote_path.startswith("~"):
            stdin, stdout, stderr = ssh.exec_command("echo $HOME")
            home_dir = stdout.read().decode().strip()
            remote_path = remote_path.replace("~", home_dir, 1)
        
        # 원격 파일 정보 확인
        try:
            remote_stat = sftp.stat(remote_path)
            remote_size = remote_stat.st_size
            print(f"📦 원격 파일 크기: {remote_size / 1024 / 1024:.2f} MB")
        except FileNotFoundError:
            print(f"❌ 원격 파일을 찾을 수 없습니다: {remote_path}")
            sftp.close()
            ssh.close()
            sys.exit(1)
        
        # 로컬 파일과 비교
        if local_path.exists():
            local_size = local_path.stat().st_size
            if local_size == remote_size:
                print(f"✅ 로컬 파일이 이미 최신 상태입니다. (크기 일치: {local_size / 1024 / 1024:.2f} MB)")
                sftp.close()
                ssh.close()
                return True
            else:
                print(f"🔄 로컬 파일 크기가 다릅니다. 업데이트 필요...")
                print(f"   로컬: {local_size / 1024 / 1024:.2f} MB")
        
        # 파일 다운로드
        print("⬇️  파일 다운로드 중...")
        sftp.get(remote_path, str(local_path))
        
        # 다운로드 확인
        if local_path.exists():
            new_size = local_path.stat().st_size
            print(f"✅ 다운로드 완료! ({new_size / 1024 / 1024:.2f} MB)")
        else:
            print("❌ 다운로드 실패!")
            sys.exit(1)
        
        # 연결 종료
        sftp.close()
        ssh.close()
        print("🔌 서버 연결 종료")
        
        return True
        
    except paramiko.AuthenticationException:
        print("❌ 인증 실패: 사용자명 또는 비밀번호가 올바르지 않습니다.")
        sys.exit(1)
    except paramiko.SSHException as e:
        print(f"❌ SSH 연결 오류: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"❌ 오류 발생: {e}")
        sys.exit(1)


if __name__ == "__main__":
    print("=" * 50)
    print("🤖 YOLO 모델 동기화 스크립트")
    print("=" * 50)
    print()
    
    success = sync_yolo_model()
    
    print()
    if success:
        print("🎉 모델 동기화 완료!")
    print("=" * 50)
