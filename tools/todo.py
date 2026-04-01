"""Todo tool — schedule tasks for the agent to do in the future.

Allows the LLM agent to schedule one-shot or recurring tasks by
communicating with the todo_manager runnable via ROS 2 topics.

Actions: add, remove, list
"""

import json
import time
import uuid
from dataclasses import dataclass

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from andr import BaseAgentTool
from std_msgs.msg import String


class TodoTool(BaseAgentTool):
    TOOL_NAME = "todo"
    TOOL_DESCRIPTION = (
        "Schedule a prompt to be sent to you (the agent) at a future time. "
        "When the todo fires, you will receive the prompt as a new task and "
        "must assess your situation, reason about how to accomplish it, and "
        "use your tools to carry it out. Use this like setting a reminder — "
        "describe WHAT needs to be done and WHY, not specific tool calls. "
        "Supports one-shot (do X in 5 minutes) and recurring (do X every hour). "
        "Can also list or remove scheduled todos."
    )
    TOOL_PARAMETERS = [
        {
            "name": "action",
            "type": "string",
            "required": True,
            "description": (
                "What to do: 'add' to schedule a new task, "
                "'remove' to cancel a scheduled task by ID, "
                "'list' to see all scheduled tasks."
            ),
        },
        {
            "name": "prompt",
            "type": "string",
            "required": False,
            "description": (
                "The prompt you will receive when this todo fires. "
                "Describe the task in natural language — what needs to "
                "be done and why. You will reason about it when the "
                "time comes. Required for 'add' action."
            ),
        },
        {
            "name": "delay_seconds",
            "type": "number",
            "required": False,
            "description": (
                "How many seconds from now to fire the task. "
                "Default: 0 (fire immediately on next tick). "
                "Examples: 300 = 5 minutes, 3600 = 1 hour, 86400 = 1 day."
            ),
        },
        {
            "name": "interval_seconds",
            "type": "number",
            "required": False,
            "description": (
                "If set, the task repeats every N seconds after first firing. "
                "Omit or set to null/0 for a one-shot task. "
                "Examples: 600 = every 10 min, 3600 = every hour."
            ),
        },
        {
            "name": "context",
            "type": "string",
            "required": False,
            "description": "Optional context string passed along with the task.",
        },
        {
            "name": "id",
            "type": "string",
            "required": False,
            "description": "Todo ID to remove. Required for 'remove' action.",
        },
    ]
    TOOL_CATEGORY = "scheduling"
    TOOL_TAGS = ["todo", "schedule", "timer", "recurring"]

    @dataclass
    class ParamsType:
        action: str = "list"
        prompt: str = ""
        delay_seconds: float = 0.0
        interval_seconds: float = 0.0
        context: str = ""
        id: str = ""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self._cb_group = ReentrantCallbackGroup()
        self._pending_responses: dict[str, dict | None] = {}

        # Publishers for requests
        self._add_pub = self.create_publisher(
            String, "/todo_manager/add_request", 10,
        )
        self._remove_pub = self.create_publisher(
            String, "/todo_manager/remove_request", 10,
        )
        self._list_pub = self.create_publisher(
            String, "/todo_manager/list_request", 10,
        )

        # Subscribers for responses
        self.create_subscription(
            String, "/todo_manager/add_response",
            self._on_add_response, 10, callback_group=self._cb_group,
        )
        self.create_subscription(
            String, "/todo_manager/remove_response",
            self._on_remove_response, 10, callback_group=self._cb_group,
        )
        self.create_subscription(
            String, "/todo_manager/list_response",
            self._on_list_response, 10, callback_group=self._cb_group,
        )

    def _execute(self, params: ParamsType, goal_handle) -> dict:
        action = params.action.lower().strip()

        if action == "add":
            return self._do_add(params)
        elif action == "remove":
            return self._do_remove(params)
        elif action == "list":
            return self._do_list()
        else:
            return {
                "status": "error",
                "message": f"Unknown action '{action}'. Use 'add', 'remove', or 'list'.",
            }

    def _do_add(self, params: ParamsType) -> dict:
        if not params.prompt.strip():
            return {"status": "error", "message": "prompt is required for 'add' action"}

        request_id = str(uuid.uuid4())[:8]
        payload = {
            "prompt": params.prompt,
            "delay_seconds": params.delay_seconds,
            "interval_seconds": params.interval_seconds if params.interval_seconds > 0 else None,
            "context": params.context,
            "request_id": request_id,
        }

        return self._send_and_wait(
            self._add_pub, payload, request_id, "add_response"
        )

    def _do_remove(self, params: ParamsType) -> dict:
        if not params.id.strip():
            return {"status": "error", "message": "id is required for 'remove' action"}

        request_id = str(uuid.uuid4())[:8]
        payload = {
            "id": params.id,
            "request_id": request_id,
        }

        return self._send_and_wait(
            self._remove_pub, payload, request_id, "remove_response"
        )

    def _do_list(self) -> dict:
        request_id = str(uuid.uuid4())[:8]
        payload = {"request_id": request_id}

        return self._send_and_wait(
            self._list_pub, payload, request_id, "list_response"
        )

    def _send_and_wait(
        self, publisher, payload: dict, request_id: str, response_type: str,
        timeout: float = 5.0,
    ) -> dict:
        """Publish a request and wait for the matching response."""
        self._pending_responses[request_id] = None

        msg = String()
        msg.data = json.dumps(payload)
        publisher.publish(msg)

        deadline = time.time() + timeout
        while time.time() < deadline:
            resp = self._pending_responses.get(request_id)
            if resp is not None:
                del self._pending_responses[request_id]
                return {"status": "done", **resp}
            time.sleep(0.05)

        self._pending_responses.pop(request_id, None)
        return {
            "status": "error",
            "message": "Timeout waiting for todo_manager response. Is it running?",
        }

    # ── Response handlers ────────────────────────────────────────────────

    def _handle_response(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        request_id = data.get("request_id", "")
        if request_id in self._pending_responses:
            self._pending_responses[request_id] = data

    def _on_add_response(self, msg: String):
        self._handle_response(msg)

    def _on_remove_response(self, msg: String):
        self._handle_response(msg)

    def _on_list_response(self, msg: String):
        self._handle_response(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TodoTool()
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
