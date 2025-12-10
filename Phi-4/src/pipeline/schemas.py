from pydantic import BaseModel, Field


class ReasoningProblem(BaseModel):
    """
    Phi-4 논문 스타일의 추론 문제 데이터 스키마입니다.
    구조화된 데이터는 모델이 논리를 학습하는 데 큰 도움을 줍니다.
    """

    topic: str = Field(..., description="문제의 주제 (예: Physics, Math)")
    difficulty: str = Field(..., description="난이도 (예: Undergraduate)")
    question: str = Field(..., description="생성된 문제의 본문")
    reasoning_steps: list[str] = Field(
        ..., description="정답 도출을 위한 단계별 사고 과정 (Chain of Thought)"
    )
    answer: str = Field(..., description="최종 정답")

    def to_markdown(self) -> str:
        """터미널에서 보기 좋게 마크다운으로 변환해주는 함수입니다."""
        steps = "\n".join(
            [f"{i+1}. {step}" for i, step in enumerate(self.reasoning_steps)]
        )
        return (
            f"### Topic: {self.topic} ({self.difficulty})\n\n"
            f"**Q:** {self.question}\n\n"
            f"**Reasoning:**\n{steps}\n\n"
            f"**A:** {self.answer}"
        )
