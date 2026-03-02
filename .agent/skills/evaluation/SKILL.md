---
name: evaluation
description: This skill should be used when the user asks to "evaluate agent performance", "build test framework", "measure agent quality", "create evaluation rubrics", or mentions LLM-as-judge, multi-dimensional evaluation, agent testing, or quality gates for agent pipelines.
---

# Evaluation Methods for Agent Systems

Evaluation of agent systems requires different approaches than traditional software or even standard language model applications. Agents make dynamic decisions, are non-deterministic between runs, and often lack single correct answers. Effective evaluation must account for these characteristics while providing actionable feedback.

## When to Activate

Use this skill when:

- Building or extending test harnesses for ROS 2 nodes
- Evaluating whether a new actuator node produces correct motor commands
- Designing CI pipelines for hardware-in-the-loop vs simulation testing
- Creating quality gates before deploying to RPi4 hardware
- Measuring tracking accuracy or actuator response metrics

## Core Concepts

### Multi-Dimensional Evaluation for Robotics

For ClumsyRobo003, evaluate along these dimensions:

| Dimension           | What to Measure                      | Test Location                     |
| ------------------- | ------------------------------------ | --------------------------------- |
| **Correctness**     | Angles match expected values         | `test_serial_motor_actuator.py`   |
| **State Machine**   | IDLE → TRACKING transitions          | Same                              |
| **Protocol**        | JSON framing, baud rate              | `SerialMotorController` unit test |
| **Latency**         | Command publish → serial write delay | Timing asserts in tests           |
| **Fault Tolerance** | ERROR state on serial failure        | Mocked serial failure test        |
| **Demo Scenario**   | Human detected → 90° commanded       | Integration test assertion        |

### Test Hierarchy for This Project

```
Level 1: Unit Tests (pure Python, no ROS)
  └── SerialMotorController.send() output format
  └── Angle clamping: clamp(95+50, 0, 180) == 145

Level 2: Integration Tests (ROS 2, simulation_mode=True)
  └── test_serial_motor_actuator.py  ← currently implemented
  └── test_tracking.py               ← existing test

Level 3: Hardware-in-the-Loop (RPi4 + real serial device)
  └── Manual test with motor attached
  └── Verify physical motor response

Level 4: System Test (full pipeline)
  └── USB camera → YOLO → tracker → serial motor
  └── Human detection → motor centers at 90°
```

### Evaluation Rubric: Actuator Node

For each new actuator implementation, validate:

- [ ] **Starts in IDLE state** — no commands before hardware ready
- [ ] **Transitions to TRACKING** — on first valid JSON command
- [ ] **Center position correct** — `center_angle=90.0` produces 90° physically
- [ ] **Angle clamping works** — inputs beyond [0°, 180°] are clamped
- [ ] **Demo mode correct** — `pan_range_deg=0.0` always yields 90°
- [ ] **Stop is safe** — `_stop()` centers motors, doesn't hold last angle
- [ ] **Serial failure → ERROR** — connection drop sets state to ERROR
- [ ] **Status publishes motor fields** — `last_angles`, `serial_port`, `simulation_mode`

### Simulation-First Testing

Always test with `simulation_mode=True` first:

```python
# Correct pattern (from test_serial_motor_actuator.py)
actuator.set_parameters([
    RosParameter('simulation_mode', RosParameter.Type.BOOL, True),
    RosParameter('pan_range_deg',   RosParameter.Type.DOUBLE, 0.0),
])
```

This allows CI to run without hardware while catching logic bugs.

### Continuous Evaluation Pattern

For this project's CI (when set up):

```bash
# Fast gate (< 30s, no hardware)
python3 test/test_tracking.py
python3 test/test_serial_motor_actuator.py

# Hardware gate (RPi4 required)
ros2 launch yolo_rpi_core yolo_tracking.launch.py \
    use_real_motor:=true simulation_mode:=false
# → Verify /tracking/status shows TRACKING and angles ≈ 90°
```

### Common Evaluation Pitfalls

- ❌ **Testing only happy path** — also test serial disconnection mid-run
- ❌ **Determinism assumption** — ROS timing is non-deterministic; use timeouts, not sleep counts
- ❌ **Ignoring state machine** — verify state transitions, not just final output
- ❌ **Hardware-only testing** — always maintain a simulation path for CI
- ❌ **Single assertion** — test all dimensions (angles, state, status fields)

## Integration with ClumsyRobo003

Key test files:

- `yolo_rpi_core/test/test_tracking.py` — pipeline integration (centroid/bytetrack)
- `yolo_rpi_core/test/test_serial_motor_actuator.py` — motor actuator simulation

Extend test coverage by:

1. Adding unit tests for `SerialMotorController` angle formatting
2. Adding fault injection test (mock serial raise `SerialException`)
3. Adding timing assertion (command latency < 100ms)

## References

- [Source: muratcankoylan/Agent-Skills-for-Context-Engineering](https://github.com/muratcankoylan/Agent-Skills-for-Context-Engineering/tree/main/skills/evaluation)
- [ROS 2 Testing Guide](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Testing-Main.html)
