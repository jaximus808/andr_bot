#!/usr/bin/env python3
"""Start the andr_bot ANDR project.

Reads configuration from andr.config.yaml and launches the full stack:
  - Core (hidden): tool_manager, prompt_manager, task_manager, agent, brain
  - Managers: map_server, etc. (auto-discovered from managers/)
  - Tools: walk, spin, navigate_to_point, etc. (auto-discovered from tools/)
  - Inputs: web_ui, etc. (auto-discovered from inputs/)
  - Runnables: standalone processes (auto-discovered from runnables/)

Usage:
    python start.py
"""

import glob
import importlib.util
import multiprocessing
import os
import sys
import time

import yaml


def load_config():
    config_path = os.path.join(os.path.dirname(__file__), "andr.config.yaml")
    if not os.path.exists(config_path):
        return {}
    with open(config_path) as f:
        return yaml.safe_load(f) or {}


def discover_modules(directory):
    """Find all .py files in a directory that aren't __init__.py."""
    pattern = os.path.join(os.path.dirname(__file__), directory, "*.py")
    return [
        f for f in glob.glob(pattern)
        if not os.path.basename(f).startswith("_")
    ]


def run_module(filepath):
    """Import and run a module's main() function in a subprocess."""
    spec = importlib.util.spec_from_file_location("_mod", filepath)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    if hasattr(mod, "main"):
        mod.main()
    else:
        print(f"Warning: {filepath} has no main() function, skipping.")


def main():
    config = load_config()
    llm = config.get("llm", {})
    agent = config.get("agent", {})
    ui = config.get("ui", {})
    brain = config.get("brain", {})
    memory = config.get("memory", {})

    # Build andr start args (core stack only — no UI, tools come from local dirs)
    start_args = [
        "andr", "start",
        "--backend", llm.get("backend", "ollama"),
        "--model", llm.get("model", "llama3.2"),
        "--host", llm.get("host", "http://localhost:11434"),
        "--temperature", str(llm.get("temperature", 0.2)),
        "--max-iterations", str(agent.get("max_iterations", 20)),
        "--ui-port", str(ui.get("port", 8080)),
        "--wander-interval", str(brain.get("wander_interval_sec", 60.0)),
        "--no-ui",   # UI is handled by inputs/web_ui.py
    ]

    if not brain.get("enabled", True):
        start_args.append("--no-brain")

    if brain.get("enable_wander", False):
        start_args.append("--enable-wander")

    if not brain.get("resume_preempted", True):
        start_args.append("--no-resume")

    # Set UI port as env var for inputs/web_ui.py
    os.environ["ANDR_UI_PORT"] = str(ui.get("port", 8080))

    # Set memory config as env vars for tools that use ChromaDB
    os.environ["ANDR_MEMORY_BACKEND"] = memory.get("backend", "chroma")
    os.environ["ANDR_MEMORY_PERSIST_PATH"] = os.path.expanduser(
        memory.get("persist_path", "/tmp/andr_memory")
    )
    os.environ["ANDR_MEMORY_COLLECTION"] = memory.get("collection_name", "andr")
    os.environ["ANDR_MEMORY_EMBEDDING_MODEL"] = memory.get("embedding_model", "all-MiniLM-L6-v2")
    os.environ["ANDR_MEMORY_TOP_K"] = str(memory.get("top_k", 4))

    # Discover managers, tools, inputs, and runnables
    manager_files = discover_modules("managers")
    tool_files = discover_modules("tools")
    input_files = discover_modules("inputs")
    runnable_files = discover_modules("runnables")

    # Launch managers first (map_server, etc.) — tools may depend on them
    procs = []
    for f in manager_files:
        fname = os.path.basename(f)
        print(f"  Launching manager: {fname}")
        p = multiprocessing.Process(target=run_module, args=(f,), daemon=True)
        p.start()
        procs.append(p)

    # Launch runnables (todo_manager, etc.) — tools may depend on them
    for f in runnable_files:
        fname = os.path.basename(f)
        print(f"  Launching runnable: {fname}")
        p = multiprocessing.Process(target=run_module, args=(f,), daemon=True)
        p.start()
        procs.append(p)

    if manager_files or runnable_files:
        time.sleep(2)  # Give managers/runnables time to register services

    # Launch tools
    for f in tool_files:
        fname = os.path.basename(f)
        print(f"  Launching tool: {fname}")
        p = multiprocessing.Process(target=run_module, args=(f,), daemon=True)
        p.start()
        procs.append(p)

    # Launch inputs (web_ui, etc.)
    for f in input_files:
        fname = os.path.basename(f)
        print(f"  Launching input: {fname}")
        p = multiprocessing.Process(target=run_module, args=(f,), daemon=True)
        p.start()
        procs.append(p)

    # Run andr start (this blocks — starts the hidden core stack)
    sys.argv = start_args
    from andr.cli import main as cli_main
    cli_main()


if __name__ == "__main__":
    main()
