"""Save-memory tool — persists facts, observations, or outcomes to the
robot's long-term memory via the memory_manager node.

Saved memories are searchable later via query_knowledge_base.
"""

import json
import time
import uuid
import threading
from dataclasses import dataclass

import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from andr_msgs.action import ExecuteSkill
from andr import BaseAgentTool


class SaveMemoryTool(BaseAgentTool):
    TOOL_NAME = "save_memory"
    TOOL_DESCRIPTION = (
        "Saves a fact, observation, or outcome to the robot's persistent memory. "
        "Saved memories are searchable later via query_knowledge_base."
    )
    TOOL_PARAMETERS = [
        {
            "name": "content",
            "type": "string",
            "required": True,
            "description": "The text to remember. Be specific and self-contained.",
        },
        {
            "name": "source",
            "type": "string",
            "required": False,
            "description": (
                "Origin tag, e.g. 'user_instruction', 'observation', "
                "'task_outcome'. Defaults to 'agent'."
            ),
        },
    ]
    TOOL_CATEGORY = "memory"
    TOOL_TAGS = ["memory", "storage", "knowledge"]

    @dataclass
    class ParamsType:
        content: str
        source: str = "agent"

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Publisher: send save requests to memory_manager
        self._save_pub = self.create_publisher(
            String, "/memory_manager/save", 10
        )

        # Subscriber: receive confirmations from memory_manager
        self._result_event = threading.Event()
        self._last_result: dict | None = None
        self._result_sub = self.create_subscription(
            String, "/memory_manager/save_result", self._on_result, 10
        )

        self._pending_request_id: str | None = None

    def _on_result(self, msg: String):
        """Handle save confirmations from the memory_manager."""
        try:
            result = json.loads(msg.data)
            if result.get("request_id") == self._pending_request_id:
                self._last_result = result
                self._result_event.set()
        except json.JSONDecodeError:
            pass

    def _execute(self, params: ParamsType, goal_handle) -> dict:
        content = params.content
        source = params.source

        self.get_logger().info(
            f"[SAVE_MEMORY] Saving memory — source={source}, "
            f"content={content[:80]}{'…' if len(content) > 80 else ''}"
        )

        # Publish feedback: starting
        feedback = ExecuteSkill.Feedback()
        feedback.status = "saving to memory"
        feedback.progress = 0.0
        goal_handle.publish_feedback(feedback)

        # Build and publish save request
        request_id = str(uuid.uuid4())
        self._pending_request_id = request_id
        self._result_event.clear()
        self._last_result = None

        msg = String()
        msg.data = json.dumps({
            "content": content,
            "source": source,
            "request_id": request_id,
        })
        self._save_pub.publish(msg)

        # Wait for confirmation (up to 5 seconds)
        confirmed = self._result_event.wait(timeout=5.0)

        if goal_handle.is_cancel_requested:
            return {"status": "cancelled"}

        if confirmed and self._last_result:
            result = self._last_result
            if result.get("status") == "saved":
                self.get_logger().info(
                    f"[SAVE_MEMORY] Confirmed — id={result.get('id', '?')[:8]}…"
                )
                feedback.status = "memory saved"
                feedback.progress = 1.0
                goal_handle.publish_feedback(feedback)
                return {
                    "status": "saved",
                    "content": content,
                    "source": source,
                    "id": result.get("id", ""),
                }
            else:
                error = result.get("error", "unknown error")
                self.get_logger().error(f"[SAVE_MEMORY] Manager error: {error}")
                return {"status": "error", "error": error}

        # Timeout — manager may not be running
        self.get_logger().warning(
            "[SAVE_MEMORY] No confirmation from memory_manager (timeout)"
        )
        return {
            "status": "saved_unconfirmed",
            "content": content,
            "source": source,
            "note": "memory_manager did not confirm within timeout",
        }


def main(args=None):
    rclpy.init(args=args)
    node = SaveMemoryTool()
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
