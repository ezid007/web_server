import os
from typing import Optional
from langchain_ollama import ChatOllama
from langchain_core.prompts import ChatPromptTemplate
from src.pipeline.schemas import ReasoningProblem


class SyntheticDataGenerator:
    def __init__(self, model_name: str = "qwen2.5:7b"):
        """
        로컬 LLM(Ollama)을 사용하여 합성 데이터를 생성하는 생성기입니다.
        """
        print(f"Loading Model: {model_name}...")

        # temperature=0.7: 창의적이지만 너무 엉뚱하지 않은 문제를 만들기 위함
        self.llm = ChatOllama(
            model=model_name, temperature=0.7, format="json"  # JSON 포맷 강제 (필수)
        )

        # Phi-4 논문의 'Textbook Quality' 원칙을 반영한 시스템 프롬프트
        system_prompt = (
            "You are an expert textbook author creating synthetic training data for a Large Language Model (Phi-4). "
            "Your goal is to transform raw text into high-quality reasoning problems.\n\n"
            "Follow these principles:\n"
            "1. **Nuance**: Create complex scenarios, not simple summaries.\n"
            "2. **Chain-of-Thought**: Ensure the solution requires step-by-step deduction.\n"
            "3. **Format**: Output strictly in JSON matching the requested schema."
        )

        self.prompt = ChatPromptTemplate.from_messages(
            [
                ("system", system_prompt),
                (
                    "human",
                    "Here is the raw seed text:\n{seed_text}\n\nCreate a reasoning problem based on this text.",
                ),
            ]
        )

        # Pydantic 스키마를 이용해 출력을 구조화
        self.chain = self.prompt | self.llm.with_structured_output(ReasoningProblem)

    def generate_problem(self, seed_text: str) -> Optional[ReasoningProblem]:
        """시드 텍스트를 받아 추론 문제를 생성합니다."""
        print("🚀 Generating synthetic data...")
        try:
            return self.chain.invoke({"seed_text": seed_text})
        except Exception as e:
            print(f"❌ Error generating data: {e}")
            return None


# 테스트 실행 코드
if __name__ == "__main__":
    # 테스트용 시드 데이터 생성 (파일이 없을 경우)
    seed_path = "data/seeds/test_seed.txt"
    if not os.path.exists(seed_path):
        with open(seed_path, "w") as f:
            f.write(
                "Archimedes' principle states that the upward buoyant force that is exerted on a body immersed in a fluid, "
                "whether fully or partially, is equal to the weight of the fluid that the body displaces."
            )

    # 생성기 실행
    generator = SyntheticDataGenerator()

    with open(seed_path, "r") as f:
        result = generator.generate_problem(f.read())

    if result:
        print("\n" + "=" * 50)
        print("✅ Generated Data Result:")
        print("=" * 50)
        print(result.to_markdown())
