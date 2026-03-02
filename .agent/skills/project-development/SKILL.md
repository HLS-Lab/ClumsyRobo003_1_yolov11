---
name: project-development
description: This skill should be used when the user asks to "start an LLM project", "design batch pipeline", "evaluate task-model fit", "structure agent project", or mentions pipeline architecture, agent-assisted development, cost estimation, or choosing between LLM and traditional approaches.
---

# Project Development Methodology

This skill covers the principles for identifying tasks suited to LLM processing, designing effective project architectures, and iterating rapidly using agent-assisted development. The methodology applies whether building a batch processing pipeline, a multi-agent research system, or an interactive agent application.

## When to Activate

Use this skill when:

- Starting a new AI/agent project and need to structure it properly
- Deciding whether to use LLMs vs traditional code for a task
- Designing a batch processing or streaming pipeline
- Evaluating task-model fit before committing to an approach
- Choosing between single-agent and multi-agent architectures
- Estimating cost and scale for production deployment

## Core Concepts

### Task-Model Fit Recognition

Before building, evaluate whether the task genuinely requires an LLM:

| Task Type                      | LLM Fit   | Rationale                                 |
| ------------------------------ | --------- | ----------------------------------------- |
| Structured rule-based logic    | ❌ Poor   | Deterministic code is faster and cheaper  |
| Natural language understanding | ✅ Strong | LLMs excel at ambiguous NL tasks          |
| Data extraction from docs      | ✅ Strong | LLMs handle unstructured formats          |
| Math/computation               | ❌ Poor   | Use deterministic tools; LLMs hallucinate |
| Code generation/review         | ✅ Strong | Strong pattern matching on code           |

### The Manual Prototype Step

Always manually prototype the core task before automating:

1. Pick 10–20 representative examples from your dataset
2. Process them manually to define "what good looks like"
3. Identify edge cases and failure modes
4. Only then design the automated pipeline

### Pipeline Architecture

For this ROS 2 robotics project, the pipeline pattern maps naturally:

```
Input Source → Processing Node → Output Sink
(v4l2_camera) → (yolo_node + tracker_node) → (actuator_node)
```

Key design principles:

- **Decouple stages** via message queues (ROS topics)
- **Each node has one responsibility** (detection, tracking, actuation)
- **Isolate hardware I/O** in dedicated adapter classes (`SerialMotorController`)
- **Use simulation modes** to test without hardware

### File System as State Machine

Use the filesystem to persist state across pipeline stages:

- Configuration in YAML files (`tracking_params.yaml`)
- Logs for debugging serial communication
- Test artifacts for CI validation

### Structured Output Design

Always define output schemas before coding:

```json
{
  "target_id": 0,
  "error_x": -0.375,
  "error_y": 0.125,
  "class_name": "person",
  "confidence": 0.85,
  "timestamp": 1234567890.123
}
```

### Agent-Assisted Development

Leverage agents for:

- **Boilerplate generation**: ROS node scaffolding, test harnesses
- **Protocol porting**: Extracting serial protocol from reference code
- **Documentation**: Architecture diagrams, README updates
- **Test generation**: Integration tests without hardware (simulation mode)

## Practical Guidance

### For This Project (ClumsyRobo003)

- **Start simple**: Always test with `DummyActuatorNode` before connecting hardware
- **Use simulation_mode=True** in `SerialMotorActuatorNode` for CI
- **Validate protocol first**: Use `serial_test.py` patterns before integrating
- **Keep nodes decoupled**: Actuator nodes should be swappable without changing tracker
- **Document the schema**: JSON message schemas are contracts between nodes

### Anti-Patterns to Avoid

- ❌ Hardcoding device paths (use ROS parameters + YAML)
- ❌ Blocking serial I/O on the main thread (use reader threads)
- ❌ Testing only with real hardware (always have simulation mode)
- ❌ Monolithic nodes (separate detection, tracking, actuation)
- ❌ Skipping the prototype step before building the pipeline

## Integration with ClumsyRobo003

This skill is particularly relevant for:

- Porting motor control from `control_v1` → `serial_motor_actuator_node`
- Designing new actuator backends (servo, stepper, CAN bus)
- Scaling the pipeline to multi-camera or multi-robot setups
- Evaluating whether to add LLM-based scene understanding

## References

- [ROS 2 Node Design Patterns](https://docs.ros.org/en/jazzy/)
- [Source: muratcankoylan/Agent-Skills-for-Context-Engineering](https://github.com/muratcankoylan/Agent-Skills-for-Context-Engineering/tree/main/skills/project-development)
