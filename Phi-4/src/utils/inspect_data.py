import json
import textwrap


def inspect_jsonl(file_path, num_samples=3):
    """JSONL 파일의 내용을 보기 좋게 출력합니다."""
    print(f"🔍 Inspecting top {num_samples} samples from: {file_path}\n")

    with open(file_path, "r", encoding="utf-8") as f:
        for i, line in enumerate(f):
            if i >= num_samples:
                break

            data = json.loads(line)

            print(f"📝 Sample #{i+1}")
            print("=" * 60)
            print(
                f"📌 Topic: {data.get('topic', 'N/A')} ({data.get('difficulty', 'N/A')})"
            )
            print("-" * 60)
            print(f"❓ Question:\n{textwrap.fill(data.get('question', ''), width=80)}")
            print("-" * 60)
            print("🧠 Reasoning Steps:")
            for idx, step in enumerate(data.get("reasoning_steps", [])):
                print(
                    f"  {idx+1}. {textwrap.fill(step, width=76, initial_indent='', subsequent_indent='     ')}"
                )
            print("-" * 60)
            print(f"💡 Answer:\n{textwrap.fill(data.get('answer', ''), width=80)}")
            print("=" * 60 + "\n")


if __name__ == "__main__":
    inspect_jsonl("data/synthetic/train.jsonl")
