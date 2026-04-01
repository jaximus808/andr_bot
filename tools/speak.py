"""Speak tool — converts text to speech and plays it through audio output.

Uses pyttsx3 (espeak-ng backend) for offline text-to-speech synthesis.
"""

import time
from dataclasses import dataclass

import rclpy
from rclpy.executors import MultiThreadedExecutor
import pyttsx3

from andr_msgs.action import ExecuteSkill
from andr import BaseAgentTool


class SpeakTool(BaseAgentTool):
    TOOL_NAME = "speak"
    TOOL_DESCRIPTION = "Convert text to speech and play it through the robot's audio output"
    TOOL_PARAMETERS = [
        {"name": "text", "type": "string", "required": True,
         "description": "The text to speak aloud"},
        {"name": "rate", "type": "integer", "required": False,
         "description": "Speech rate in words per minute (default 150)"},
        {"name": "volume", "type": "number", "required": False,
         "description": "Volume level from 0.0 to 1.0 (default 1.0)"},
    ]
    TOOL_CATEGORY = "communication"
    TOOL_TAGS = ["speech", "audio", "tts"]

    @dataclass
    class ParamsType:
        text: str = ""
        rate: int = 150
        volume: float = 1.0

    def _execute(self, params: ParamsType, goal_handle) -> dict:
        text = params.text
        rate = params.rate
        volume = max(0.0, min(1.0, params.volume))

        if not text.strip():
            return {"status": "error", "message": "No text provided"}

        self.get_logger().info(
            f"[SPEAK] text='{text[:80]}...' rate={rate} volume={volume}"
            if len(text) > 80
            else f"[SPEAK] text='{text}' rate={rate} volume={volume}"
        )

        # Publish initial feedback
        feedback = ExecuteSkill.Feedback()
        feedback.status = "synthesizing speech"
        feedback.progress = 0.1
        goal_handle.publish_feedback(feedback)

        if goal_handle.is_cancel_requested:
            return {"status": "cancelled"}

        # Initialize TTS engine, synthesize and play
        engine = pyttsx3.init()
        engine.setProperty("rate", rate)
        engine.setProperty("volume", volume)

        feedback.status = "speaking"
        feedback.progress = 0.5
        goal_handle.publish_feedback(feedback)

        engine.say(text)
        engine.runAndWait()
        engine.stop()

        feedback.status = "done speaking"
        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)

        self.get_logger().info(f"[SPEAK] Done — spoke {len(text)} characters")
        return {
            "status": "done",
            "text": text,
            "characters": len(text),
        }


def main(args=None):
    rclpy.init(args=args)
    node = SpeakTool()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
