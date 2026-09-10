# BlueROV2 Visual Following Control & MAVLink Integration

Linux-side control, safety, and simulation-integration subsystem for a BlueROV2 visual-following system.

The controller receives computer-vision observations over UDP, converts image and distance errors into motion commands, and sends them to ArduSub through MAVLink `MANUAL_CONTROL`.

This repository contains the **control / vehicle-side subsystem** of the TÜBİTAK 2209-A supported graduation project:

**BlueROV2 LED-Based Target Tracking System & Distributed Co-Simulation**

The corresponding perception repository is:

➡️ [bluerov2-led-tracking-opencv](https://github.com/eranaydogan/bluerov2-led-tracking-opencv)

---

## System Architecture

```text
Unity live visual environment
        ↓
OpenCV LED perception
        ↓
target error + relative distance
        ↓
UDP observation packet
        ↓
Linux controller
        ↓
validation + control + safety logic
        ↓
MAVLink MANUAL_CONTROL
        ↓
ArduSub SITL
        ↓
Gazebo BlueROV2
        ↓
MAVLink vehicle pose
        ↓
UDP pose bridge
        ↓
Unity follower camera / visual feedback
```

The distributed architecture separates visual perception from vehicle control:

- **Windows / Unity / OpenCV** handles image acquisition and target perception.
- **Linux / ArduSub / Gazebo** handles vehicle control and dynamics.
- UDP connects the perception and pose-feedback subsystems.

---

## Main Controller

The current main controller is:

```text
scripts/06_live_udp_to_mavlink_controller.py
```

It receives observations containing fields such as:

```text
valid
face_id
pattern_accuracy
distance_confidence
error_norm
estimated_distance
held_observation
udp_seq
```

The controller converts:

```text
horizontal image error → yaw command
relative distance error → forward/backward command
```

Vertical motion is kept neutral during normal visual following:

```text
y = 0
z = 500
```

---

## Control Logic

### Forward / Distance Control

The controller uses:

```text
distance_error = estimated_distance - desired_distance
```

A positive distance error produces forward motion, while a target closer than the desired following distance can produce limited reverse motion.

The current implementation includes:

- forward deadband,
- configurable desired following distance,
- forward command limiting,
- limited reverse authority,
- distance-dependent forward boost,
- yaw-priority forward gating,
- distance-dependent gate scaling,
- far-target chase behavior.

Forward gating reduces translational motion when the target is far from the image center, allowing the vehicle to prioritize alignment before moving aggressively forward.

For distant targets, the gate can be relaxed to prevent the leader from escaping while the follower is still aligning.

---

### Yaw Control

Yaw control uses horizontal image error:

```text
error_x = error_norm[0]
```

The controller supports:

- proportional yaw control,
- optional derivative damping,
- yaw deadband,
- yaw command limiting,
- distance-dependent yaw gain,
- filtered image error before derivative calculation,
- yaw reverse protection.

The derivative term is optional:

```text
--k-yaw-d 0
```

keeps pure proportional behavior.

A non-zero value enables P+D yaw control.

---

## Observation Validation

Before generating motion commands, incoming observations are checked for:

- valid detection state,
- expected LED face,
- minimum temporal-pattern accuracy,
- minimum distance confidence,
- valid image-error fields,
- finite numerical values,
- plausible estimated-distance range,
- plausible horizontal image error.

Mission-level sanity checks can reject measurements that are unrealistically close, far away, or too far outside the expected visual region.

---

## Held and Lost Observations

Temporary perception loss is handled explicitly.

Controller states include:

```text
TRACK
HELD_DECAY
INVALID_DECAY
INVALID_STOP
PACKET_TIMEOUT
NO_PACKET
```

Held observations are treated as stale measurements and do not update the yaw derivative state or distance controller.

During invalid observations, commands decay before transitioning to a hard stop.

Packet timeout or prolonged invalid input causes immediate neutral control.

---

## Command Smoothing and Safety

Raw controller outputs are not sent directly to the vehicle.

The command pipeline includes:

```text
target command
      ↓
EMA smoothing
      ↓
rate limiting
      ↓
MANUAL_CONTROL
```

Safety behavior includes:

- neutral command before arming,
- heartbeat-based arm confirmation,
- invalid-observation decay,
- hard stop after prolonged invalid input,
- packet-timeout stop,
- command saturation,
- sequence-gap warnings,
- STOP on keyboard interrupt,
- STOP before exit,
- automatic disarm when the controller armed the vehicle.

Neutral command:

```text
x = 0
y = 0
z = 500
r = 0
```

---

## MAVLink and ArduSub Integration

The project uses MAVLink `MANUAL_CONTROL` as the standard vehicle-control interface.

Validated axis convention:

```text
x > 0   → forward
x < 0   → backward

r > 0   → yaw right
r < 0   → yaw left

z = 500 → neutral vertical command
```

The controller communicates with ArduSub SITL and drives the Gazebo BlueROV2 simulation.

RC override is not used as the main control interface.

---

## Gazebo → Unity Pose Bridge

The current pose bridge is:

```text
scripts/08_mavlink_pose_to_unity.py
```

It reads:

```text
LOCAL_POSITION_NED
ATTITUDE
```

from MAVLink and forwards vehicle pose to Unity using a compact 9-float UDP packet.

The bridge supports:

- relative-position origin locking,
- initial-body-frame transformation,
- NED-to-Unity coordinate conversion,
- yaw / roll / pitch forwarding,
- axis inversion and swapping,
- configurable pose rate,
- independent horizontal and vertical scaling.

Example data flow:

```text
Gazebo / ArduSub
      ↓
MAVLink LOCAL_POSITION_NED + ATTITUDE
      ↓
08_mavlink_pose_to_unity.py
      ↓
UDP pose packet
      ↓
Unity CV_Test_Camera
```

This bridge allows simulated vehicle motion to affect the live Unity visual input used by the perception system.

---

## Diagnostic Scripts

| Script | Purpose |
|---|---|
| `01_mavlink_connection_check.py` | Verify MAVLink heartbeat and telemetry |
| `02_manual_control_stop_test.py` | Validate neutral `MANUAL_CONTROL` and safe arm/disarm |
| `03_manual_control_axis_test.py` | Verify forward/backward and yaw mappings |
| `04_udp_dry_run_controller.py` | Test UDP observation processing without vehicle control |
| `05_udp_to_mavlink_controller_safe.py` | Earlier safe UDP-to-MAVLink reference controller |
| `06_live_udp_to_mavlink_controller.py` | Current visual-following controller |
| `07_manual_control_threshold_test.py` | Determine practical MANUAL_CONTROL response thresholds |
| `08_mavlink_pose_to_unity.py` | Stream Gazebo/ArduSub pose back to Unity |
| `09_yaw_only_pulse_test.py` | Isolated low-authority yaw diagnostics |
| `10_forward_only_pulse_test.py` | Isolated forward/backward diagnostics |
| `11_unity_pose_axis_probe.py` | Verify Unity UDP pose-axis mapping |

The isolated yaw test keeps:

```text
x = 0
y = 0
z = 500
```

and applies only `r`, while the forward test keeps yaw disabled and applies only `x`.

---

## Installation

Create a Python environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
```

Install the required package:

```bash
pip install -r requirements.txt
```

Current dependency:

```text
pymavlink==2.4.49
```

---

## Basic Controller Run

The controller listens for vision observations on UDP port `5005` by default.

Arms-off test:

```bash
python scripts/06_live_udp_to_mavlink_controller.py \
  --runtime 30 \
  --packet-timeout 1.0 \
  --log-csv logs/control_test.csv
```

This processes live observations and generates commands without arming the vehicle.

After the simulation and safety chain have been verified:

```bash
python scripts/06_live_udp_to_mavlink_controller.py \
  --runtime 30 \
  --packet-timeout 1.0 \
  --arm \
  --log-csv logs/control_test.csv
```

More advanced tuning options are available for:

```text
desired distance
forward gain
yaw P/D gains
distance-dependent yaw gain
forward gating
distance-dependent forward boost
far-target chase behavior
deadbands
command smoothing
rate limiting
observation confidence thresholds
measurement sanity checks
```

---

## Pose Bridge Example

Example MAVLink-to-Unity bridge:

```bash
python scripts/08_mavlink_pose_to_unity.py \
  --mavlink udpin:127.0.0.1:14552 \
  --unity-ip <WINDOWS_IP> \
  --unity-port 5008 \
  --body-frame-relative
```

Optional independent motion scaling:

```text
--scale-horizontal
--scale-vertical
```

can be used to align simulated Gazebo motion with the Unity visualization.

---

## Development Progression

```text
MAVLink connection
        ↓
MANUAL_CONTROL safety test
        ↓
axis mapping
        ↓
UDP dry-run controller
        ↓
safe UDP-to-MAVLink integration
        ↓
controller smoothing and rate limiting
        ↓
Gazebo / ArduSub motion validation
        ↓
live UDP controller
        ↓
Gazebo-to-Unity pose bridge
        ↓
yaw-only closed-loop validation
        ↓
forward motion diagnostics
        ↓
distance-gated visual following
```

The latest controller stage focuses on making forward motion dependent on both relative distance and visual alignment while preserving conservative yaw and safety behavior.

---

## Related Perception Repository

### [BlueROV2 LED-Based Visual Tracking & Perception](https://github.com/eranaydogan/bluerov2-led-tracking-opencv)

The perception subsystem provides:

- live Unity Game View capture,
- OpenCV LED detection,
- LED-pair selection,
- image-center error,
- relative-distance estimation,
- observation validation,
- UDP observation streaming,
- mission replay and trajectory analysis.

The perception repository also contains the current visual demo and trajectory-analysis results.

---

## Documentation

Additional development and setup information is available under:

```text
docs/
├── setup_and_run.md
├── progress_log.md
├── test_log.md
└── next_steps.md
```

These documents preserve detailed integration tests, tuning history, and simulation setup notes.

---

## Current Scope and Limitations

The current system is an experimental research prototype.

Current limitations include:

- horizontal visual following is substantially more developed than vertical control,
- normal tracking keeps vertical command fixed at `z=500`,
- controller gains remain simulation-dependent,
- relative distance comes from monocular LED pixel spacing rather than metric localization,
- coordinate and scale alignment between Gazebo and Unity requires configuration,
- current control is based primarily on the BACK LED face,
- robust multi-face tracking and recovery/search behavior remain future extensions.

The controller is intentionally conservative around stale, invalid, or implausible visual measurements.

---

## Project Context

**Graduation Project**  
**Funded by TÜBİTAK 2209-A**  
**Role: Project Lead**

The broader project explores distributed simulation, visual perception, closed-loop control, and autonomous following for underwater robotic systems using BlueROV2.
