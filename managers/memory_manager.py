"""Memory management service node.

Manages a ChromaDB-backed persistent memory store that the agent can
write to (via the save_memory tool) and read from (via query_knowledge_base).

Both the agent's built-in RAG pipeline and this manager point at the same
ChromaDB persist directory, so writes here are immediately visible to reads.
"""

import json
import os
import pathlib
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import chromadb
from chromadb.config import Settings

DEFAULT_PERSIST_DIR = "/tmp/andr_memory"
COLLECTION_NAME = "robot_memory"


class MemoryManager(Node):
    def __init__(self):
        super().__init__("memory_manager")

        self.declare_parameter("persist_dir", DEFAULT_PERSIST_DIR)
        self._persist_dir = (
            self.get_parameter("persist_dir")
            .get_parameter_value()
            .string_value
        )

        pathlib.Path(self._persist_dir).mkdir(parents=True, exist_ok=True)

        # Initialise ChromaDB client (file-backed)
        self._client = chromadb.Client(
            Settings(
                chroma_db_impl="duckdb+parquet",
                persist_directory=self._persist_dir,
                anonymized_telemetry=False,
            )
        )
        self._collection = self._client.get_or_create_collection(
            name=COLLECTION_NAME,
            metadata={"hnsw:space": "cosine"},
        )

        # Topic: tool publishes save requests here
        self._save_sub = self.create_subscription(
            String, "/memory_manager/save", self._on_save, 10
        )

        # Topic: manager publishes save confirmations
        self._result_pub = self.create_publisher(
            String, "/memory_manager/save_result", 10
        )

        self.get_logger().info(
            f"MemoryManager ready — persist_dir={self._persist_dir}, "
            f"collection={COLLECTION_NAME} "
            f"(existing docs: {self._collection.count()})"
        )

    # ------------------------------------------------------------------
    # Save callback
    # ------------------------------------------------------------------
    def _on_save(self, msg: String):
        """Handle a save request published by the save_memory tool."""
        try:
            payload = json.loads(msg.data)
            content = payload["content"]
            source = payload.get("source", "agent")
            request_id = payload.get("request_id", "")

            doc_id = str(uuid.uuid4())
            metadata = {
                "source": source,
                "timestamp": time.time(),
            }

            self._collection.add(
                documents=[content],
                metadatas=[metadata],
                ids=[doc_id],
            )

            self.get_logger().info(
                f"[MEMORY] Saved id={doc_id[:8]}… source={source} "
                f"({self._collection.count()} total docs)"
            )

            # Publish confirmation
            result = String()
            result.data = json.dumps(
                {"status": "saved", "id": doc_id, "request_id": request_id}
            )
            self._result_pub.publish(result)

        except Exception as e:
            self.get_logger().error(f"[MEMORY] Save failed: {e}")
            result = String()
            result.data = json.dumps(
                {"status": "error", "error": str(e),
                 "request_id": payload.get("request_id", "")}
            )
            self._result_pub.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = MemoryManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
