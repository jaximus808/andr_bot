"""Remember tool — stores information into the robot's ChromaDB memory.

Allows the LLM agent to persist knowledge (facts, observations, user
preferences, spatial info, etc.) so it can be recalled later via the
built-in query_knowledge_base tool.

Memory configuration is read from env vars set by start.py (sourced from
andr.config.yaml → memory section):
  ANDR_MEMORY_BACKEND, ANDR_MEMORY_PERSIST_PATH, ANDR_MEMORY_COLLECTION,
  ANDR_MEMORY_EMBEDDING_MODEL, ANDR_MEMORY_TOP_K
"""

import os
from dataclasses import dataclass

import rclpy
from rclpy.executors import MultiThreadedExecutor

from andr import BaseAgentTool
from andr.runtime.agent.memory import create_memory


class RememberTool(BaseAgentTool):
    TOOL_NAME = "remember"
    TOOL_DESCRIPTION = (
        "Store a piece of information into long-term memory. Use this to "
        "remember facts, observations, user preferences, locations, object "
        "positions, task outcomes, or anything worth recalling later. "
        "Stored memories can be retrieved with query_knowledge_base."
    )
    TOOL_PARAMETERS = [
        {
            "name": "text",
            "type": "string",
            "required": True,
            "description": "The information to remember. Be specific and descriptive.",
        },
        {
            "name": "source",
            "type": "string",
            "required": False,
            "description": (
                "Category or source label for the memory "
                "(e.g. 'observation', 'user_preference', 'location', 'task_result'). "
                "Default: 'agent'"
            ),
        },
    ]
    TOOL_CATEGORY = "memory"
    TOOL_TAGS = ["memory", "knowledge", "storage"]

    @dataclass
    class ParamsType:
        text: str = ""
        source: str = "agent"

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        backend = os.environ.get("ANDR_MEMORY_BACKEND", "chroma")
        persist_path = os.environ.get("ANDR_MEMORY_PERSIST_PATH", "/tmp/andr_memory")
        collection_name = os.environ.get("ANDR_MEMORY_COLLECTION", "andr")
        embedding_model = os.environ.get("ANDR_MEMORY_EMBEDDING_MODEL", "all-MiniLM-L6-v2")
        self._memory = create_memory(
            backend,
            persist_path=persist_path,
            collection_name=collection_name,
            embedding_model=embedding_model,
        )

    def _execute(self, params: ParamsType, goal_handle) -> dict:
        if not params.text.strip():
            return {"status": "error", "message": "Cannot store empty text"}

        metadata = {"source": params.source}

        self._memory.add(params.text, metadata=metadata)

        self.get_logger().info(
            f"[REMEMBER] Stored memory (source='{params.source}'): "
            f"{params.text[:80]}{'...' if len(params.text) > 80 else ''}"
        )

        count = self._memory._col.count()

        return {
            "status": "done",
            "message": f"Stored in memory (total memories: {count})",
            "text_stored": params.text,
            "source": params.source,
        }


def main(args=None):
    rclpy.init(args=args)
    node = RememberTool()
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
