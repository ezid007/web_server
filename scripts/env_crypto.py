"""
환경변수 암호화/복호화 유틸리티
민감한 정보를 안전하게 관리합니다.
"""

import os
from pathlib import Path
from cryptography.fernet import Fernet
from dotenv import dotenv_values

# 프로젝트 루트 디렉토리
BASE_DIR = Path(__file__).parent.parent

# 파일 경로
ENV_FILE = BASE_DIR / ".env"
ENV_ENCRYPTED_FILE = BASE_DIR / ".env.encrypted"
KEY_FILE = BASE_DIR / ".env.key"

# 암호화 설정
# True: 모든 환경변수 암호화 / False: SENSITIVE_KEYS만 암호화
ENCRYPT_ALL = True

# 암호화할 민감한 키 목록 (ENCRYPT_ALL=False일 때만 사용)
SENSITIVE_KEYS = [
    "TURTLEBOT_PASSWORD",
    "REMOTE_MODEL_PASSWORD",
    "GOOGLE_API_KEY",
    "WEATHER_API_KEY",
]


def generate_key() -> bytes:
    """새 암호화 키 생성"""
    return Fernet.generate_key()


def load_key() -> bytes:
    """키 파일에서 암호화 키 로드"""
    if not KEY_FILE.exists():
        raise FileNotFoundError(
            f"암호화 키 파일이 없습니다: {KEY_FILE}\n"
            "새 키를 생성하려면: python -m src.env_crypto generate-key"
        )
    return KEY_FILE.read_bytes().strip()


def save_key(key: bytes) -> None:
    """암호화 키를 파일에 저장"""
    KEY_FILE.write_bytes(key)
    print(f"✅ 암호화 키 저장됨: {KEY_FILE}")
    print("⚠️  이 파일은 절대 Git에 커밋하지 마세요!")


def encrypt_value(value: str, key: bytes) -> str:
    """값 암호화"""
    f = Fernet(key)
    return f.encrypt(value.encode()).decode()


def decrypt_value(encrypted_value: str, key: bytes) -> str:
    """값 복호화"""
    f = Fernet(key)
    return f.decrypt(encrypted_value.encode()).decode()


def encrypt_env_file() -> None:
    """
    .env 파일의 민감한 값들을 암호화하여 .env.encrypted로 저장
    """
    if not ENV_FILE.exists():
        raise FileNotFoundError(f".env 파일이 없습니다: {ENV_FILE}")
    
    key = load_key()
    env_values = dotenv_values(ENV_FILE)
    
    encrypted_lines = []
    for k, v in env_values.items():
        # ENCRYPT_ALL이면 모든 값 암호화, 아니면 SENSITIVE_KEYS만
        should_encrypt = ENCRYPT_ALL or (k in SENSITIVE_KEYS)
        if should_encrypt and v:
            encrypted_v = encrypt_value(v, key)
            encrypted_lines.append(f"{k}=ENC:{encrypted_v}")
            print(f"🔐 암호화됨: {k}")
        else:
            encrypted_lines.append(f"{k}={v if v else ''}")
    
    ENV_ENCRYPTED_FILE.write_text("\n".join(encrypted_lines))
    print(f"\n✅ 암호화된 환경변수 저장됨: {ENV_ENCRYPTED_FILE}")


def decrypt_env_file() -> None:
    """
    .env.encrypted 파일을 복호화하여 .env로 저장
    """
    if not ENV_ENCRYPTED_FILE.exists():
        raise FileNotFoundError(f"암호화된 파일이 없습니다: {ENV_ENCRYPTED_FILE}")
    
    key = load_key()
    
    decrypted_lines = []
    for line in ENV_ENCRYPTED_FILE.read_text().splitlines():
        if "=" in line and not line.startswith("#"):
            k, v = line.split("=", 1)
            if v.startswith("ENC:"):
                encrypted_v = v[4:]  # "ENC:" 제거
                decrypted_v = decrypt_value(encrypted_v, key)
                decrypted_lines.append(f"{k}={decrypted_v}")
                print(f"🔓 복호화됨: {k}")
            else:
                decrypted_lines.append(line)
        else:
            decrypted_lines.append(line)
    
    ENV_FILE.write_text("\n".join(decrypted_lines))
    print(f"\n✅ 복호화된 환경변수 저장됨: {ENV_FILE}")


def load_env_with_decryption() -> dict:
    """
    암호화된 환경변수를 복호화하여 딕셔너리로 반환
    (서버 실행 시 사용)
    """
    # 키 파일이 있으면 암호화된 파일 사용
    if KEY_FILE.exists() and ENV_ENCRYPTED_FILE.exists():
        key = load_key()
        env_values = dotenv_values(ENV_ENCRYPTED_FILE)
        
        result = {}
        for k, v in env_values.items():
            if v and v.startswith("ENC:"):
                result[k] = decrypt_value(v[4:], key)
            else:
                result[k] = v
        return result
    
    # 키 파일이 없으면 일반 .env 파일 사용
    elif ENV_FILE.exists():
        return dict(dotenv_values(ENV_FILE))
    
    return {}


def show_encrypted_status() -> None:
    """현재 암호화 상태 표시"""
    print("📋 암호화 상태 확인\n")
    print(f"  .env 파일: {'✅ 있음' if ENV_FILE.exists() else '❌ 없음'}")
    print(f"  .env.encrypted 파일: {'✅ 있음' if ENV_ENCRYPTED_FILE.exists() else '❌ 없음'}")
    print(f"  .env.key 파일: {'✅ 있음' if KEY_FILE.exists() else '❌ 없음'}")
    
    if ENV_ENCRYPTED_FILE.exists():
        print("\n📝 암호화된 키 목록:")
        for line in ENV_ENCRYPTED_FILE.read_text().splitlines():
            if "=ENC:" in line:
                k = line.split("=")[0]
                print(f"  🔐 {k}")


# CLI 실행
if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 2:
        print("사용법:")
        print("  python scripts/env_crypto.py generate-key  # 새 암호화 키 생성")
        print("  python scripts/env_crypto.py encrypt       # .env → .env.encrypted")
        print("  python scripts/env_crypto.py decrypt       # .env.encrypted → .env")
        print("  python scripts/env_crypto.py status        # 현재 상태 확인")
        sys.exit(1)
    
    command = sys.argv[1]
    
    if command == "generate-key":
        if KEY_FILE.exists():
            print(f"⚠️  키 파일이 이미 존재합니다: {KEY_FILE}")
            confirm = input("덮어쓰시겠습니까? (y/N): ")
            if confirm.lower() != "y":
                print("취소됨")
                sys.exit(0)
        save_key(generate_key())
        
    elif command == "encrypt":
        encrypt_env_file()
        
    elif command == "decrypt":
        decrypt_env_file()
        
    elif command == "status":
        show_encrypted_status()
        
    else:
        print(f"알 수 없는 명령어: {command}")
        sys.exit(1)
