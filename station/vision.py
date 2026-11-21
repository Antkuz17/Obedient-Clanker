"""Image tasks on a forward frame (Vultr VISION_MODEL): read, solve, describe, photo.

    text = await ask(ai, "read", jpeg, question="what does the sign say?")

Answers are written to be spoken: short, no markdown.
"""
from __future__ import annotations

import re

from .ai import AI

SYSTEM = ("You are the eyes of a small drone. Your answer is read aloud: at most two "
          "short sentences, no markdown, no lists, no LaTeX.")

PROMPTS = {
    "read": "Read out the main text visible in this image, exactly. If there is no readable "
            "text, say so.",
    "solve": "There is a maths or logic problem in this image. Solve it and give the final "
             "answer first, then one line of working.",
    "describe": "Describe what is in front of the drone in one sentence.",
    "pointing": "A person in this image is pointing at something. Name the single object "
                "they are pointing at, in one to three words, nothing else. If nobody is "
                "pointing, reply NONE.",
    "caption": "Caption this frame for a search index: list the distinct objects you can see, "
               "with colours, and where they are (left/centre/right). One line, no prose.",
}


def clean(text: str) -> str:
    text = re.sub(r"<think>.*?</think>", "", text, flags=re.S)
    text = re.sub(r"[*_#`$\\]", "", text)
    return " ".join(text.split())


async def ask(ai: AI, task: str, jpeg: bytes, question: str = "") -> str:
    prompt = PROMPTS.get(task, PROMPTS["describe"])
    if question and task not in ("caption", "pointing"):
        prompt += f' The user asked: "{question}".'
    return clean(await ai.vision(prompt, jpeg, system=SYSTEM))
