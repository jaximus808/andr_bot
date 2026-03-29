# my_robot_2 — Claude Development Guide

This is a mobile robot project built on the **ANDR framework** (`andr` pip package).
ANDR is an LLM-powered robotics toolkit on ROS 2 where an agent controls a robot
through modular, decoupled tools.

## Quick Reference

```
pip package:   andr==0.1.0
source:        /home/observer/andr/
core:          /home/observer/andr/andr_core/
pip pkg src:   /home/observer/andr/pip/andr/
```

---

## Project Structure

```
my_robot_2/
├── andr.config.yaml          # LLM backend, agent, brain, UI settings
├── start.py                  # Entry point — auto-discovers and launches everything
├── tools/                    # Agent tools (BaseAgentTool subclasses)
│   ├── walk.py               # Forward/backward movement via /cmd_vel
│   ├── spin.py               # In-place rotation via /cmd_vel
│   └── navigate_to_point.py  # Waypoint navigation via Nav2
├── managers/                 # Long-running services
│   ├── map_server.py         # Map/SLAM/waypoint management (SQLite)
│   └── migrations/           # DB schema evolution (001_, 002_, etc.)
├── inputs/                   # Input sources (BaseInputSource subclasses)
│   └── web_ui.py             # FastAPI + WebSocket dashboard bridge
├── runnables/                # Standalone processes (auto-discovered)
├── ui/static/                # Frontend HTML/CSS/JS
│   ├── index.html            # Main web dashboard
│   └── rviz.html             # RViz visualization page
└── ros/                      # ROS-related configs (currently empty)
```

---

## How It Works

### Startup Flow

`python start.py` does the following:
1. Reads `andr.config.yaml`
2. Spawns managers (map_server) as subprocesses, waits 2s for registration
3. Spawns tools (walk, spin, navigate_to_point) as subprocesses
4. Spawns inputs (web_ui) as subprocesses
5. Spawns any runnables
6. Calls `andr start` CLI with config args, which launches the hidden core stack:
   - **tool_manager** (C++ binary) — tool registry and dispatch
   - **prompt_manager** — system prompt versioning
   - **task_manager** — task routing
   - **agent_server** — LLM ReAct loop
   - **task_brain** (optional) — priority queue, preemption, scheduling

### Task Execution Flow

```
User types in Web UI
  → WebSocket message to web_ui.py
    → BaseInputSource.send_task(prompt)
      → /task_manager/execute (TaskGoal action)
        → task_brain (optional priority/preemption)
          → /agent/prompt (Agent action)
            → LLM decides which tool(s) to call
              → /tool_manager/execute (ExecuteSkill action)
                → /tools/{name} action server (walk, spin, etc.)
            → Feedback streams back through the full chain
              → WebSocket → Browser UI
```

### Navigation Example

"Go to the kitchen" →
1. Agent calls `navigate_to_point(point_name="kitchen")`
2. Tool queries map_server: resolve "kitchen" → (x, y) coordinates
3. Tool sends `NavigateToPose` goal to Nav2
4. Nav2 navigates, feedback streams back to UI

---

## ANDR Framework Architecture

### Core Principles

1. **The agent is tool-agnostic** — it discovers tools at runtime from tool_manager. Never add sensor-specific logic to agent code.
2. **Everything is a tool** — any capability must be a registered tool. Agent finds it automatically.
3. **Input sources are bridges, not agents** — they observe events and call `send_task()`. They never bypass the task_manager pipeline.
4. **task_manager is the single entry point** — ALL tasks flow through `/task_manager/execute`.
5. **System prompt stays generic** — tool descriptions come from the registry, not hardcoded prompts.

### Layer Diagram

```
┌───────────────────────────────────────────────────────┐
│  INPUT SOURCES (BaseInputSource subclasses)            │
│  web_ui.py, future: voice, vision, SMS, API            │
│       │                                                │
│       ▼                                                │
│  task_manager  →  task_brain (priority/preemption)     │
│       │                                                │
│       ▼                                                │
│  agent_server (LLM ReAct loop, max 20 iterations)     │
│       │                                                │
│       ▼                                                │
│  tool_manager (C++ registry + dispatch)                │
│       │                                                │
│       ▼                                                │
│  TOOLS (BaseAgentTool subclasses)                      │
│  walk, spin, navigate_to_point, custom tools           │
└───────────────────────────────────────────────────────┘
```

---

## andr Pip Package — What It Provides

### Public API

```python
from andr import BaseAgentTool, BaseInputSource
```

These are the only two classes you need to build a robot.

### CLI

```bash
andr init <name>          # Scaffold a new robot project
andr start                # Launch the core stack
andr task "<prompt>"      # Send a task to the running agent
andr status               # Check running nodes
```

### `andr start` Options

```
--backend {ollama,openai}    --model <name>           --host <url>
--temperature <float>        --max-iterations <int>   --ui-port <port>
--no-ui                      --no-brain               --enable-wander
--wander-interval <float>    --no-resume
```

### Bundled Components (launched by `andr start`)

- **tool_manager** — C++ binary, registers/deregisters/dispatches tools
- **prompt_manager** — system prompt CRUD + history
- **task_manager** — task routing via `/task_manager/execute`
- **agent_server** — LLM ReAct loop (LangChain + Ollama/OpenAI)
- **task_brain** — priority scheduler with preemption
- **web UI** — FastAPI + WebSocket (optional, `--no-ui` to skip)

### Bundled ROS 2 Messages (`andr_msgs`)

```python
from andr_msgs.action import ExecuteSkill, TaskGoal, Agent
from andr_msgs.srv import RegisterTool, DeregisterTool, ListTools
from andr_msgs.srv import SaveMap, SavePoint, GetMapPoints, GetMaps
from andr_msgs.srv import SetSlamConfig, GetSlamConfig, RestartSlam
from andr_msgs.srv import GetAgentConfig, SetAgentConfig
from andr_msgs.srv import GetSystemPrompt, SetSystemPrompt, GetPromptHistory
from andr_msgs.msg import RobotSpeech
```

### Dependencies

langchain, langchain-core, langchain-ollama, langchain-openai, chromadb,
sentence-transformers, fastapi, uvicorn, websockets, pyyaml, pydantic

---

## BaseAgentTool — Creating Tools

Subclass to create a tool the LLM agent can call. It auto-registers with tool_manager.

```python
from andr import BaseAgentTool

class MyTool(BaseAgentTool):
    TOOL_NAME = "my_tool"                         # unique identifier
    TOOL_DESCRIPTION = "Does something useful"    # shown to LLM
    TOOL_PARAMETERS = [                           # LLM sees these
        {"name": "param1", "type": "string", "required": True,
         "description": "What to do"},
        {"name": "speed", "type": "number", "required": False,
         "description": "How fast", "default": 1.0},
    ]
    TOOL_CATEGORY = "general"                     # optional grouping
    TOOL_TAGS = ["movement"]                      # optional tags

    def _execute(self, params: dict, goal_handle) -> dict:
        # params is a dict with the parsed parameters
        # goal_handle lets you publish feedback and check cancellation
        #
        # Publish feedback:
        #   self._publish_feedback(goal_handle, progress=0.5, status="halfway")
        #
        # Check cancellation:
        #   if goal_handle.is_cancel_requested: ...
        #
        return {"status": "done", "result": "whatever"}
```

### Parameter Types

`"string"`, `"number"`, `"integer"`, `"boolean"`, `"array"`, `"object"`

### Optional: Typed Parameters

Define a `ParamsType` dataclass for automatic conversion from dict:

```python
from dataclasses import dataclass

class WalkTool(BaseAgentTool):
    @dataclass
    class ParamsType:
        direction: str = "forward"
        duration_s: float = 2.0
        speed: float = 0.2

    # _execute receives ParamsType instance instead of raw dict
    def _execute(self, params: ParamsType, goal_handle):
        ...
```

### Lifecycle

1. `__init__` → registers with tool_manager via `RegisterTool` service
2. Agent discovers tool via `tool_manager/list`
3. Agent calls tool → `_execute()` runs
4. On shutdown → deregisters via `DeregisterTool` service

---

## BaseInputSource — Creating Input Sources

Subclass to create something that sends tasks to the agent pipeline.

```python
from andr import BaseInputSource

class MyInput(BaseInputSource):
    SOURCE_NAME = "my_input"
    SOURCE_DESCRIPTION = "Receives tasks from somewhere"

    def __init__(self):
        super().__init__()
        # Set up your event listener (ROS subscription, HTTP server, etc.)

    # Send a task to the agent:
    # self.send_task(prompt="do something", context="optional metadata")

    # Check if a task is already running:
    # if not self.is_busy: self.send_task(...)

    # Lifecycle hooks (override as needed):
    def on_task_accepted(self, prompt): ...
    def on_task_rejected(self, prompt): ...
    def on_task_feedback(self, state, status, progress): ...
    def on_task_completed(self, prompt, success, summary): ...
```

---

## This Robot's Components

### Tools

**walk.py** — Move forward/backward
- Publishes `Twist` to `/cmd_vel` at 20 Hz
- Params: `direction` (forward/backward), `duration_s`, `speed` (m/s)
- Cancellable, streams progress feedback

**spin.py** — Rotate in place
- Publishes `Twist` angular velocity to `/cmd_vel`
- Params: `duration_s`, `speed_deg_s`, `direction` (left/right)
- Same feedback/cancellation pattern as walk

**navigate_to_point.py** — Navigate to named waypoints
- Queries map_server for point coordinates, sends `NavigateToPose` to Nav2
- Params: `point_name` (string)
- Forwards Nav2 distance feedback as progress

### Managers

**map_server.py** — Map, waypoint, and SLAM management
- SQLite database at `~/andr_maps/maps.db`
- Tables: `maps`, `points`, `slam_config`
- Services: save_map, get_maps, save_point, get_map_points, get_point_coordinates,
  set_slam_config, get_slam_config, restart_slam, get_map_with_points
- Optional integrations: SLAM Toolbox, Gazebo reset, Nav2 lifecycle

### Inputs

**web_ui.py** — Web dashboard bridge
- FastAPI + WebSocket server (port from config, default 8080)
- Routes: `GET /` (dashboard), `GET /rviz` (visualization), `WS /ws` (events)
- Subscribes to ROS topics: /robot/speech, /agent/feedback, /robot/status,
  /map, /scan, /odom, /amcl_pose (all throttled)
- Proxies browser requests to ANDR services (map, agent config, prompt, tools)
- Queue-based communication between ROS thread and async web loop

---

## Configuration (andr.config.yaml)

```yaml
llm:
  backend: ollama              # or "openai"
  model: llama3.2              # model name
  host: http://localhost:11434 # Ollama server URL
  temperature: 0.2             # LLM sampling temperature

agent:
  max_iterations: 20           # ReAct loop safety limit

brain:
  enabled: true                # task_brain (priority/preemption)
  enable_wander: false         # idle task generation
  wander_interval_sec: 60.0
  resume_preempted: true       # resume interrupted tasks

ui:
  enabled: true
  port: 8080
```

---

## Key ROS Interfaces

| Interface | Type | Path |
|---|---|---|
| Task entry | Action | `/task_manager/execute` (TaskGoal) |
| Agent prompt | Action | `/agent/prompt` (Agent) |
| Tool dispatch | Action | `/tool_manager/execute` (ExecuteSkill) |
| Individual tools | Action | `/tools/<name>` (ExecuteSkill) |
| Tool listing | Service | `tool_manager/list` (ListTools) |
| Tool registration | Service | `tool_manager/register` (RegisterTool) |
| System prompt | Service | `prompt_manager/get_system_prompt` |
| Agent config | Service | `agent/get_config`, `agent/set_config` |
| Map services | Service | `map_manager/*` |

---

## Task Brain — Priority & Preemption

```python
class Priority(IntEnum):
    IDLE = 1         # Wander/idle behavior
    SCHEDULED = 2    # Cron-like recurring tasks
    USER = 3         # User-initiated from UI
    URGENT = 4       # Safety-critical
```

Higher-priority tasks preempt lower ones. Preempted tasks can be resumed
(if `resume_preempted: true`). Scheduled tasks fire on intervals.

---

## Agent — LLM ReAct Loop

The agent runs a ReAct (Reasoning + Acting) loop:
1. Build system message with robot identity + available tools
2. For each iteration (up to `max_iterations`):
   a. Call LLM with conversation history
   b. If LLM returns tool call → execute via tool_manager → append result
   c. If LLM returns final answer → done
3. Supports both structured tool calls and raw JSON fallback
4. Memory via ChromaDB (RAG) for context-aware decisions

LLM config: backend (ollama/openai), model, temperature, host

---

## Common Patterns

### Adding a new tool

1. Create `tools/my_tool.py` subclassing `BaseAgentTool`
2. Define `TOOL_NAME`, `TOOL_DESCRIPTION`, `TOOL_PARAMETERS`
3. Implement `_execute(params, goal_handle)`
4. It will be auto-discovered by `start.py` — no other changes needed

### Adding a new input source

1. Create `inputs/my_input.py` subclassing `BaseInputSource`
2. Define `SOURCE_NAME`, `SOURCE_DESCRIPTION`
3. Call `self.send_task(prompt, context)` when events occur
4. Override `on_task_*` hooks as needed
5. Auto-discovered by `start.py`

### Adding a new manager

1. Create `managers/my_manager.py` — a regular ROS 2 node
2. Expose ROS services for the tools/inputs to call
3. Auto-discovered by `start.py` (launched before tools)

### Do NOT

- Modify agent code to support new tools (tools self-register)
- Subscribe to sensor topics from the agent (use input source bridges)
- Bypass task_manager by sending directly to `/agent/prompt`
- Hardcode tool descriptions in the system prompt (they come from registry)
- Add sensor-specific logic to the agent

---

## Build & Run

```bash
# Start the robot
cd /home/observer/andr_test/my_robot_2
python start.py

# Or use CLI directly
andr start --backend ollama --model llama3.2

# Send a task
andr task "walk forward for 3 seconds"

# Check status
andr status
```

Web UI available at `http://localhost:8080` when running.

---

## Source Code Locations

### This Project
- Config: `andr.config.yaml`
- Entry point: `start.py`
- Tools: `tools/walk.py`, `tools/spin.py`, `tools/navigate_to_point.py`
- Map manager: `managers/map_server.py`
- Web UI input: `inputs/web_ui.py`
- Frontend: `ui/static/index.html`, `ui/static/rviz.html`

### ANDR Framework (pip package source)
- Package root: `/home/observer/andr/pip/andr/`
- Base classes: `andr/tools/base_agent_tool.py`, `andr/tools/base_input_source.py`
- CLI: `andr/cli.py`
- Runtime agent: `andr/runtime/agent/`
- Runtime task manager: `andr/runtime/task_manager/`
- Templates: `andr/templates/`

### ANDR Core (ROS 2 workspace source)
- Agent ReAct loop: `/home/observer/andr/andr_core/agent/agent/agent.py`
- Skill registry: `/home/observer/andr/andr_core/agent/agent/skills.py`
- Task brain: `/home/observer/andr/andr_core/task_manager/task_manager/task_brain.py`
- Tool manager (C++): `/home/observer/andr/andr_core/tool_manager/src/tool_manager.cpp`
- Base classes: `/home/observer/andr/andr_core/andr_tools/andr_tools/`
- System prompt: `/home/observer/andr/andr_core/agent/agent/prompts/system_prompt.py`
- Launch configs: `/home/observer/andr/andr_core/andr_launch/`
