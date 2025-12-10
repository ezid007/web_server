import os
import glob
import json
from src.pipeline.generator import SyntheticDataGenerator


def main():
    # 1. 경로 설정
    seed_dir = "data/seeds"
    output_file = "data/synthetic/train.jsonl"

    # 시드 파일 목록 가져오기 (*.txt)
    seed_files = glob.glob(os.path.join(seed_dir, "**/*.txt"), recursive=True)
    print(f"📂 Found {len(seed_files)} seed files in '{seed_dir}' (including subdirectories)")

    # 2. 생성기 초기화 (Qwen 모델 로드)
    # 모델이 메모리에 한 번만 로드되므로 효율적입니다.
    generator = SyntheticDataGenerator()

    # 기존 출력 파일이 있으면 초기화 (덮어쓰기)
    if os.path.exists(output_file):
        os.remove(output_file)

    print("🚀 Starting batch generation... (This may take time)")

    success_count = 0

    # 3. 파일 순회하며 데이터 생성
    with open(output_file, "w", encoding="utf-8") as out_f:
        for i, file_path in enumerate(seed_files):
            file_name = os.path.basename(file_path)
            print(
                f"[{i+1}/{len(seed_files)}] Processing: {file_name}...",
                end=" ",
                flush=True,
            )

            try:
                # 시드 텍스트 읽기
                with open(file_path, "r", encoding="utf-8") as f:
                    seed_text = f.read()

                # Qwen에게 문제 생성 요청
                result = generator.generate_problem(seed_text)

                if result:
                    # 결과(JSON)를 한 줄로 변환하여 쓰기 (JSONL 포맷)
                    # Pydantic 모델 -> dict -> json string
                    json_line = result.model_dump_json()
                    out_f.write(json_line + "\n")

                    print("✅ Done")
                    success_count += 1
                else:
                    print("⚠️ Skipped (Generation Failed)")

            except Exception as e:
                print(f"❌ Error: {e}")

    # 4. 결과 리포트
    print("=" * 50)
    print(f"🎉 Generation Complete!")
    print(f"📊 Success Rate: {success_count}/{len(seed_files)}")
    print(f"💾 Saved to: {output_file}")
    print("=" * 50)


if __name__ == "__main__":
    main()
