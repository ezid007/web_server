import torch
from transformers import AutoTokenizer, AutoModelForCausalLM, BitsAndBytesConfig
from peft import PeftModel


def main():
    # ==========================================
    # 1. 모델 경로 설정
    # ==========================================
    base_model_name = "Qwen/Qwen2.5-3B-Instruct"  # 원본(학생) 모델
    adapter_path = "models/phi-4-3b-reasoning"  # 우리가 가르친 지식(어댑터)

    print(f"🔄 Loading Base Model: {base_model_name}")

    # ==========================================
    # 2. 4비트 양자화 설정 (메모리 절약)
    # ==========================================
    # 학습 때와 똑같이 4비트로 불러와야 8GB VRAM에서 돕니다.
    bnb_config = BitsAndBytesConfig(
        load_in_4bit=True,
        bnb_4bit_quant_type="nf4",
        bnb_4bit_use_double_quant=True,
        bnb_4bit_compute_dtype=torch.float16,
    )

    # ==========================================
    # 3. 모델 로딩 및 병합
    # ==========================================
    # 1) 원본 모델 로드
    base_model = AutoModelForCausalLM.from_pretrained(
        base_model_name, quantization_config=bnb_config, device_map="auto"
    )
    tokenizer = AutoTokenizer.from_pretrained(base_model_name)

    # 2) 학습된 어댑터(LoRA) 장착
    # 원본 모델 옆에 우리가 학습시킨 얇은 지식층을 끼워 넣습니다.
    print(f"➕ Merging Adapter: {adapter_path}")
    model = PeftModel.from_pretrained(base_model, adapter_path)

    # ==========================================
    # 4. 추론 테스트 (새로운 질문)
    # ==========================================
    # 학습 데이터에는 없지만, 약리학적 지식과 추론이 필요한 질문을 던져봅니다.
    test_question = (
        "A patient with hypertension is prescribed a calcium channel blocker. "
        "Explain how this drug works to lower blood pressure, specifically focusing on vascular smooth muscle cells. "
        "Also, predict what would happen if the patient develops severe acidosis."
    )

    # 학습 때 썼던 프롬프트 포맷(ChatML)을 그대로 지켜야 성능이 잘 나옵니다.
    prompt = (
        f"<|im_start|>system\nYou are a helpful assistant capable of complex reasoning.<|im_end|>\n"
        f"<|im_start|>user\n{test_question}<|im_end|>\n"
        f"<|im_start|>assistant\nLet's think step by step.\n"
    )

    # 텍스트 -> 토큰 변환
    inputs = tokenizer(prompt, return_tensors="pt").to("cuda")

    print("\n🤖 Generating Answer...\n")
    print("=" * 50)

    # 생성 시작
    with torch.no_grad():
        outputs = model.generate(
            **inputs,
            max_new_tokens=512,  # 최대 512토큰까지 생성
            temperature=0.7,  # 창의성 (0.7 정도가 적당)
            top_p=0.9,  # 다양한 표현 사용
            do_sample=True,  # 확률적 생성 사용
            eos_token_id=tokenizer.eos_token_id,
            pad_token_id=tokenizer.eos_token_id,
        )

    # 토큰 -> 텍스트 변환 (결과 출력)
    generated_text = tokenizer.decode(outputs[0], skip_special_tokens=True)

    # 프롬프트 뒷부분(생성된 답변)만 깔끔하게 출력
    # (system, user 부분은 잘라내고 assistant 답변만 보여줌)
    answer_start = generated_text.find("Let's think step by step.")
    final_answer = generated_text[answer_start:]

    print(final_answer)
    print("=" * 50)


if __name__ == "__main__":
    main()
