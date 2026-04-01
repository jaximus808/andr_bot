"""Todo Manager — persistent scheduled task runner.

Manages a list of todos (one-shot and recurring) that fire agent tasks
at the scheduled time. Persists todos to a JSON file so they survive
restarts.

Communication with the todo tool happens via ROS 2 services:
  - /todo_manager/add       (std_srvs/Trigger-like via JSON in request)
  - /todo_manager/remove    (remove by ID)
  - /todo_manager/list      (return all todos)

When a todo fires, it sends a task through the standard pipeline
via /task_manager/execute (TaskGoal action).
"""

import json
import os
import threading
import time
import uuid
from datetime import datetime

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from andr_msgs.action import TaskGoal

# We use rcl_interfaces SetParameters as a generic string-in/string-out
# service pattern won't work. Instead, use a simple topic-based RPC
# with std_msgs for maximum compatibility without custom msgs.
from std_msgs.msg import String


TODOS_DIR = os.path.expanduser("~/andr_data")
TODOS_FILE = os.path.join(TODOS_DIR, "todos.json")


class TodoManager(Node):
    def __init__(self):
        super().__init__("todo_manager")

        self._cb_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()

        # Persistence
        os.makedirs(TODOS_DIR, exist_ok=True)
        self._todos: dict[str, dict] = {}
        self._load_todos()

        # Action client to send tasks through the pipeline
        self._task_client = ActionClient(
            self, TaskGoal, "/task_manager/execute",
            callback_group=self._cb_group,
        )

        # ── ROS service topics ──────────────────────────────────────────
        # Request/response via paired topics (tool publishes request,
        # manager publishes response). Uses JSON payloads.
        self._add_sub = self.create_subscription(
            String, "/todo_manager/add_request",
            self._handle_add, 10, callback_group=self._cb_group,
        )
        self._add_pub = self.create_publisher(
            String, "/todo_manager/add_response", 10,
        )

        self._remove_sub = self.create_subscription(
            String, "/todo_manager/remove_request",
            self._handle_remove, 10, callback_group=self._cb_group,
        )
        self._remove_pub = self.create_publisher(
            String, "/todo_manager/remove_response", 10,
        )

        self._list_sub = self.create_subscription(
            String, "/todo_manager/list_request",
            self._handle_list, 10, callback_group=self._cb_group,
        )
        self._list_pub = self.create_publisher(
            String, "/todo_manager/list_response", 10,
        )

        # ── Scheduler loop ──────────────────────────────────────────────
        self._timer = self.create_timer(
            5.0, self._tick, callback_group=self._cb_group,
        )

        count = len(self._todos)
        self.get_logger().info(
            f"TodoManager ready — loaded {count} todo(s) from {TODOS_FILE}"
        )

    # ── Persistence ──────────────────────────────────────────────────────

    def _load_todos(self):
        if not os.path.isfile(TODOS_FILE):
            return
        try:
            with open(TODOS_FILE) as f:
                data = json.load(f)
            for todo in data:
                self._todos[todo["id"]] = todo
        except Exception as e:
            self.get_logger().error(f"Failed to load todos: {e}")

    def _save_todos(self):
        try:
            with open(TODOS_FILE, "w") as f:
                json.dump(list(self._todos.values()), f, indent=2)
        except Exception as e:
            self.get_logger().error(f"Failed to save todos: {e}")

    # ── Handlers ─────────────────────────────────────────────────────────

    def _handle_add(self, msg: String):
        """Add a new todo. JSON payload:
        {
            "prompt": "do something",
            "delay_seconds": 300,         # fire in 5 minutes (one-shot)
            "interval_seconds": null,     # or repeat every N seconds
            "context": "optional context",
            "request_id": "uuid"          # for matching response
        }
        """
        try:
            req = json.loads(msg.data)
        except json.JSONDecodeError:
            self._publish_response(self._add_pub, {
                "success": False, "error": "Invalid JSON",
            })
            return

        now = time.time()
        delay = req.get("delay_seconds", 0) or 0
        interval = req.get("interval_seconds")
        prompt = req.get("prompt", "")
        context = req.get("context", "")
        request_id = req.get("request_id", "")

        if not prompt.strip():
            self._publish_response(self._add_pub, {
                "success": False, "error": "Empty prompt",
                "request_id": request_id,
            })
            return

        todo_id = str(uuid.uuid4())[:8]
        fire_at = now + delay

        todo = {
            "id": todo_id,
            "prompt": prompt,
            "context": context,
            "created_at": datetime.now().isoformat(),
            "fire_at": fire_at,
            "interval_seconds": interval,
            "recurring": interval is not None and interval > 0,
            "fire_count": 0,
            "active": True,
        }

        with self._lock:
            self._todos[todo_id] = todo
            self._save_todos()

        fire_dt = datetime.fromtimestamp(fire_at).strftime("%Y-%m-%d %H:%M:%S")
        self.get_logger().info(
            f"[TODO] Added '{todo_id}': \"{prompt[:60]}\" — "
            f"fires at {fire_dt}"
            f"{f', repeats every {interval}s' if todo['recurring'] else ' (one-shot)'}"
        )

        self._publish_response(self._add_pub, {
            "success": True,
            "id": todo_id,
            "fire_at": fire_dt,
            "recurring": todo["recurring"],
            "request_id": request_id,
        })

    def _handle_remove(self, msg: String):
        try:
            req = json.loads(msg.data)
        except json.JSONDecodeError:
            self._publish_response(self._remove_pub, {
                "success": False, "error": "Invalid JSON",
            })
            return

        todo_id = req.get("id", "")
        request_id = req.get("request_id", "")

        with self._lock:
            if todo_id in self._todos:
                removed = self._todos.pop(todo_id)
                self._save_todos()
                self.get_logger().info(
                    f"[TODO] Removed '{todo_id}': \"{removed['prompt'][:60]}\""
                )
                self._publish_response(self._remove_pub, {
                    "success": True, "id": todo_id,
                    "request_id": request_id,
                })
            else:
                self._publish_response(self._remove_pub, {
                    "success": False, "error": f"Todo '{todo_id}' not found",
                    "request_id": request_id,
                })

    def _handle_list(self, msg: String):
        try:
            req = json.loads(msg.data)
        except json.JSONDecodeError:
            req = {}

        request_id = req.get("request_id", "")

        with self._lock:
            todos = list(self._todos.values())

        formatted = []
        for t in todos:
            fire_dt = datetime.fromtimestamp(t["fire_at"]).strftime("%Y-%m-%d %H:%M:%S")
            formatted.append({
                "id": t["id"],
                "prompt": t["prompt"],
                "fire_at": fire_dt,
                "recurring": t.get("recurring", False),
                "interval_seconds": t.get("interval_seconds"),
                "fire_count": t.get("fire_count", 0),
                "active": t.get("active", True),
            })

        self._publish_response(self._list_pub, {
            "success": True,
            "todos": formatted,
            "count": len(formatted),
            "request_id": request_id,
        })

    # ── Scheduler tick ───────────────────────────────────────────────────

    def _tick(self):
        now = time.time()
        to_fire: list[dict] = []

        with self._lock:
            for todo in list(self._todos.values()):
                if not todo.get("active", True):
                    continue
                if now >= todo["fire_at"]:
                    to_fire.append(todo)

        for todo in to_fire:
            self._fire_todo(todo)

    def _build_prompt(self, todo: dict) -> str:
        """Build a rich prompt that makes the agent reason about context
        before acting — like a human receiving a reminder."""
        task = todo["prompt"]
        context = todo.get("context", "")
        todo_id = todo["id"]
        created = todo.get("created_at", "unknown")
        fire_count = todo.get("fire_count", 0)
        recurring = todo.get("recurring", False)
        now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        lines = [
            f"SCHEDULED REMINDER (todo {todo_id}):",
            f"You scheduled this task at {created}. It is now {now_str}.",
        ]

        if recurring:
            interval = todo.get("interval_seconds", 0)
            lines.append(
                f"This is a recurring task (every {interval}s, "
                f"fired {fire_count + 1} time(s) so far)."
            )

        if context:
            lines.append(f"Context: {context}")

        lines.append("")
        lines.append(f"Task: {task}")
        lines.append("")
        lines.append(
            "Before acting, assess your current situation — where you are, "
            "what state you're in, what tools and information you have "
            "available. Then figure out how to accomplish this task and "
            "carry it out using your tools."
        )

        return "\n".join(lines)

    def _fire_todo(self, todo: dict):
        todo_id = todo["id"]
        prompt = self._build_prompt(todo)

        self.get_logger().info(
            f"[TODO] Firing '{todo_id}': \"{todo['prompt'][:80]}\""
        )

        # Send task through the pipeline
        if not self._task_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                f"[TODO] task_manager unavailable — skipping '{todo_id}'"
            )
            return

        goal = TaskGoal.Goal()
        goal.prompt = prompt
        goal.context = f"scheduled_todo:{todo_id}"

        future = self._task_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f, tid=todo_id: self._on_task_sent(f, tid)
        )

        with self._lock:
            todo["fire_count"] = todo.get("fire_count", 0) + 1

            if todo.get("recurring") and todo.get("interval_seconds"):
                # Reschedule
                todo["fire_at"] = time.time() + todo["interval_seconds"]
                next_dt = datetime.fromtimestamp(todo["fire_at"]).strftime(
                    "%Y-%m-%d %H:%M:%S"
                )
                self.get_logger().info(
                    f"[TODO] Rescheduled '{todo_id}' → next at {next_dt}"
                )
            else:
                # One-shot — remove it
                self._todos.pop(todo_id, None)
                self.get_logger().info(
                    f"[TODO] One-shot '{todo_id}' completed and removed"
                )

            self._save_todos()

    def _on_task_sent(self, future, todo_id: str):
        try:
            goal_handle = future.result()
            if goal_handle and goal_handle.accepted:
                self.get_logger().info(
                    f"[TODO] Task for '{todo_id}' accepted by task_manager"
                )
            else:
                self.get_logger().warn(
                    f"[TODO] Task for '{todo_id}' rejected by task_manager"
                )
        except Exception as e:
            self.get_logger().error(
                f"[TODO] Failed to send task for '{todo_id}': {e}"
            )

    # ── Helpers ──────────────────────────────────────────────────────────

    def _publish_response(self, publisher, data: dict):
        msg = String()
        msg.data = json.dumps(data)
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TodoManager()
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
