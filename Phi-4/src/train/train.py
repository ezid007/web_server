import os

# [핵심 설정 1] 메모리 파편화 방지
# GPU 메모리가 조각나서 "공간은 남는데 큰 덩어리를 못 넣는" 현상을 방지합니다.
# OOM(Out Of Memory) 에러를 막는 첫 번째 방어선입니다.
os.environ["PYTORCH_ALLOC_CONF"] = "expandable_segments:True"

import torch
from datasets import load_dataset
from transformers import (
    AutoModelForCausalLM,  # 텍스트 생성 모델 로더
    AutoTokenizer,  # 텍스트 <-> 숫자 변환기
    BitsAndBytesConfig,  # 모델 압축(양자화) 설정 도구
)
from peft import (
    LoraConfig,
    prepare_model_for_kbit_training,
)  # LoRA(가중치 튜닝) 관련 도구
from trl import SFTTrainer, SFTConfig  # 실제 학습을 수행하는 트레이너


def main():
    # ============================================================
    # 1. 기본 설정 (Configuration)
    # ============================================================
    model_name = "Qwen/Qwen2.5-3B-Instruct"  # 학습시킬 학생 모델 (30억 파라미터)
    dataset_path = "data/synthetic/train.jsonl"  # 우리가 만든 교과서 (합성 데이터)
    output_dir = "models/phi-4-3b-reasoning"  # 학습 결과가 저장될 폴더

    # GPU 캐시 청소: 혹시 남아있을지 모를 쓰레기 데이터를 비웁니다.
    torch.cuda.empty_cache()

    # ============================================================
    # 2. 데이터 준비 (Data Preparation)
    # ============================================================
    print(f"📚 Loading dataset from {dataset_path}...")
    dataset = load_dataset("json", data_files=dataset_path, split="train")

    # 데이터를 모델이 이해할 수 있는 '대화형 프롬프트'로 변환하는 함수
    def format_instruction(sample):
        # 단계별 추론(Reasoning Steps) 리스트를 줄바꿈으로 연결하여 하나의 긴 글로 만듭니다.
        steps = "\n".join(
            [f"{i+1}. {step}" for i, step in enumerate(sample["reasoning_steps"])]
        )

        # [프롬프트 엔지니어링]
        # Phi-4 논문의 핵심인 "ChatML" 포맷을 적용합니다.
        # System: 역할 부여 / User: 질문 / Assistant: 사고 과정 + 정답
        prompt = (
            f"<|im_start|>system\nYou are a helpful assistant capable of complex reasoning.<|im_end|>\n"
            f"<|im_start|>user\n{sample['question']}<|im_end|>\n"
            f"<|im_start|>assistant\nLet's think step by step.\n\n{steps}\n\n**Answer:** {sample['answer']}<|im_end|>"
        )
        # [주의] 리스트가 아닌 문자열 자체를 반환해야 합니다.
        return prompt

    # ============================================================
    # 3. 모델 압축 설정 (Quantization)
    # ============================================================
    # 8GB VRAM에 모델을 올리기 위한 필수 설정입니다.
    bnb_config = BitsAndBytesConfig(
        load_in_4bit=True,  # 모델을 4비트로 압축해서 로딩 (용량 약 1/4로 감소)
        bnb_4bit_quant_type="nf4",  # 4비트 중에서도 성능 손실이 적은 'NF4' 방식 사용
        bnb_4bit_use_double_quant=True,  # 이중 압축 기술로 메모리를 한 번 더 절약
        bnb_4bit_compute_dtype=torch.float16,  # 연산 속도를 위해 계산은 16비트로 수행
        llm_int8_enable_fp32_cpu_offload=True,  # VRAM 부족 시 시스템 RAM(64GB)을 빌려 씀
    )

    print(f"🤖 Loading Student Model: {model_name}")
    model = AutoModelForCausalLM.from_pretrained(
        model_name,
        quantization_config=bnb_config,  # 위에서 정의한 압축 설정 적용
        device_map="auto",  # GPU/CPU 자동 할당
        use_cache=False,  # 학습 중에는 과거 기억 캐싱을 꺼서 메모리 절약
    )

    # 모델을 학습 가능한 상태로 전처리하고, Gradient Checkpointing을 켭니다.
    # Gradient Checkpointing: 중간 계산 결과를 저장 안 하고 필요할 때 다시 계산 (속도↓ 메모리 효율↑)
    model = prepare_model_for_kbit_training(model, use_gradient_checkpointing=True)

    # ============================================================
    # 4. 토크나이저 설정 (Tokenizer)
    # ============================================================
    tokenizer = AutoTokenizer.from_pretrained(model_name)
    tokenizer.pad_token = (
        tokenizer.eos_token
    )  # 문장 길이를 맞출 때 빈 공간을 EOS(문장끝) 토큰으로 채움
    tokenizer.padding_side = "right"  # 오른쪽 빈 공간을 채움

    # [핵심 수정] 메모리 폭발 방지용 길이 제한
    # 512 토큰이 넘어가면 가차 없이 자릅니다. OOM 해결의 일등공신입니다.
    tokenizer.model_max_length = 512

    # ============================================================
    # 5. LoRA 설정 (Low-Rank Adaptation)
    # ============================================================
    # 모델 전체를 학습하는 건 불가능하므로, 얇은 '어댑터'만 붙여서 학습합니다.
    peft_config = LoraConfig(
        r=16,  # 어댑터의 두께 (Rank). 높을수록 똑똑해지지만 메모리를 더 씀.
        lora_alpha=32,  # 학습 반영률. 보통 r의 2배로 설정.
        lora_dropout=0.05,  # 과적합 방지를 위해 랜덤하게 일부 뉴런을 끔.
        bias="none",
        task_type="CAUSAL_LM",
        # 학습시킬 타겟 레이어들 (Qwen 모델의 모든 주요 연산 부위)
        target_modules=[
            "q_proj",
            "k_proj",
            "v_proj",
            "o_proj",
            "gate_proj",
            "up_proj",
            "down_proj",
        ],
    )

    # ============================================================
    # 6. 학습 실행 설정 (Training Arguments)
    # ============================================================
    training_args = SFTConfig(
        output_dir=output_dir,
        # [메모리 최적화의 끝판왕 설정]
        per_device_train_batch_size=1,  # 한 번에 딱 1문제만 풉니다. (가장 안전)
        gradient_accumulation_steps=8,  # 대신 8번 푼 결과를 모아서 한 번에 업데이트 (배치 사이즈 8 효과)
        learning_rate=2e-4,  # 학습 속도 (너무 빠르면 멍청해지고, 너무 느리면 답답함)
        logging_steps=1,  # 1스텝마다 로그 출력 (실시간 확인용)
        num_train_epochs=3,  # 같은 문제집을 3번 반복 학습
        fp16=True,  # 16비트 연산 사용 (속도 향상)
        save_strategy="epoch",  # 1 Epoch 끝날 때마다 저장
        optim="paged_adamw_8bit",  # [중요] 옵티마이저도 압축해서 메모리 절약
        gradient_checkpointing=True,  # 메모리 절약 모드 활성화
        max_grad_norm=0.3,  # 학습이 튀는 것을 방지 (안전장치)
        warmup_ratio=0.03,  # 초반 3%는 천천히 학습하며 예열
        lr_scheduler_type="constant",  # 학습률을 일정하게 유지
        packing=False,  # 데이터를 꽉 채우지 않음 (메모리 안전 우선)
        dataset_text_field="text",  # 데이터셋 필드명 (형식상 필요)
    )

    # ============================================================
    # 7. 트레이너 실행 (Run Training)
    # ============================================================
    trainer = SFTTrainer(
        model=model,  # 학생 (Qwen 3B)
        train_dataset=dataset,  # 교재 (합성 데이터)
        peft_config=peft_config,  # 학습법 (LoRA)
        formatting_func=format_instruction,  # 데이터 가공 함수
        args=training_args,  # 학습 계획표
        processing_class=tokenizer,  # 통역사
    )

    print("🚀 Starting 3B Model Training (Final Fix)...")
    trainer.train()  # 실제 학습 시작!

    # 학습 완료 후 저장
    print(f"💾 Saving adapter to {output_dir}")
    trainer.model.save_pretrained(output_dir)
    tokenizer.save_pretrained(output_dir)


if __name__ == "__main__":
    main()
