---
name: filesystem-context
description: This skill should be used when the user asks to "offload context to files", "implement dynamic context discovery", "use filesystem for agent memory", "reduce context window bloat", or mentions file-based context management, tool output persistence, agent scratch pads, or just-in-time context loading.
---

# Filesystem-Based Context Engineering

The filesystem provides a single interface through which agents can flexibly store, retrieve, and update an effectively unlimited amount of context. This pattern addresses the fundamental constraint that context windows are limited while tasks often require more information than fits in a single window.

The core insight is that files enable dynamic context discovery: agents pull relevant context on demand rather than carrying everything in the context window.

## When to Activate

Use this skill when:

- Working with large codebases (like this ROS 2 project) where all files don't fit in context
- Needing to persist state across multiple agent sessions
- Coordinating sub-tasks that each need different subsets of project context
- Managing configuration across multiple nodes and launch files
- Storing intermediate results (test logs, serial communication traces)

## Core Concepts

### Static vs Dynamic Context

| Type        | Description                | When to Use                                      |
| ----------- | -------------------------- | ------------------------------------------------ |
| **Static**  | Always included in context | Core architecture, active file being edited      |
| **Dynamic** | Loaded on demand           | Node implementations, test files, config details |

For ClumsyRobo003: Keep `ARCHITECTURE.md` as static reference; load individual node files dynamically when modifying them.

### Key Patterns

#### Pattern 1: Filesystem as Scratch Pad

Use temp files for intermediate work:

```
/tmp/
├── serial_trace.log      # Captured UART frames for debugging
├── detection_samples/    # Test YOLO detection inputs
└── motor_test_results/   # Integration test outputs
```

#### Pattern 2: Plan Persistence

Store work plans in tracked files:

```
.agent/
├── rules/               # Project-specific agent rules
└── skills/              # Domain skills (this file)
```

#### Pattern 3: Configuration as Context

Use YAML files as structured context for agent tasks:

```yaml
# tracking_params.yaml — agents read this to understand node config
serial_motor_actuator_node:
  ros__parameters:
    simulation_mode: true
    center_angle: 90.0
    pan_range_deg: 0.0
```

#### Pattern 4: Dynamic Skill Loading

This `.agent/skills/` directory IS the dynamic skill loading pattern. Skills are loaded when relevant keywords appear in the task context.

#### Pattern 5: Log Persistence

Persist ROS logs and serial traces for async debugging:

```bash
# Capture serial trace to file for later analysis
ros2 topic echo /tracking/status > /tmp/status_trace.jsonl &
```

### File Organization for This Project

```
ClumsyRobo003_1_yolov11/
├── ARCHITECTURE.md          ← Static context (always relevant)
├── README.md                ← Project entry point
├── .agent/
│   ├── rules/               ← Persistent coding rules
│   └── skills/              ← This directory (dynamic skill context)
└── yolo_rpi_core/
    ├── config/              ← Node configuration context
    │   └── tracking_params.yaml
    └── yolo_rpi_core/       ← Source; load individual files as needed
```

### Token Accounting

When working on this project, prioritize loading:

1. `ARCHITECTURE.md` — always (system overview)
2. The specific node being modified (e.g., `serial_motor_actuator_node.py`)
3. Its YAML config section
4. Related test file

Avoid loading all node source files simultaneously.

## Practical Guidance for ClumsyRobo003

- Use `tracking_params.yaml` as the single source of truth for node config
- Reference `ARCHITECTURE.md` for node topology before editing launch files
- Use `/tmp/` for debugging artifacts (serial traces, test outputs)
- Store recurring coding patterns in `.agent/rules/`

## References

- [Source: muratcankoylan/Agent-Skills-for-Context-Engineering](https://github.com/muratcankoylan/Agent-Skills-for-Context-Engineering/tree/main/skills/filesystem-context)
