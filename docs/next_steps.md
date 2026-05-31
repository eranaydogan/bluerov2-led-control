Next Steps
Current Status

The control side has passed the offline video-based integration milestone.

Validated chain:

Unity recorded video
→ OpenCV video sender V2
→ UDP observation
→ Linux controller V2
→ MAVLink MANUAL_CONTROL
→ Gazebo BlueROV2 motion
→ STOP + DISARM

This proves that the communication and control pipeline works. However, it is still not true closed-loop tracking because the video input does not change when the Gazebo robot moves.

1. Preserve Current Stable Controller

The current main controller is:

scripts/06_live_udp_to_mavlink_controller.py

Do not replace it immediately. Future experiments should either:

add optional parameters,
create a new script if behavior changes significantly,
keep 05_udp_to_mavlink_controller_safe.py as a simpler reference controller.

Current important controller features:

yaw deadband,
forward deadband,
EMA smoothing,
acceleration/rate limiting,
invalid decay,
hard STOP,
CSV logging,
heartbeat-based arm/disarm check.
2. Add Control Log Analysis

Create:

scripts/08_analyze_control_log.py

Input example:

logs/control_v2_armed_test_03.csv

The script should compute:

total control samples,
time spent in each state:
TRACK,
INVALID_DECAY,
INVALID_STOP,
PACKET_TIMEOUT,
NO_PACKET,
average/max target_x,
average/max smooth_x,
average/max target_r,
average/max smooth_r,
command saturation ratio,
number of STOP transitions,
packet age statistics,
held observation ratio,
distance error range,
yaw error range.

This will make controller tuning more systematic.

3. Test with Clean Constant-ON Video

The first dynamic video was useful as a stress test, but it included blink/OFF frames and temporary occlusion.

The next vision-side video should be:

BackOnly_Dynamic_Clean_01_CONSTANT_ON.mp4

Control goal:

evaluate pure tracking and controller smoothing without blink-induced target loss.

Expected controller behavior:

more continuous TRACK,
fewer INVALID_DECAY states,
fewer INVALID_STOP states,
smoother x/r commands.

Recommended arms-off command:

python scripts/06_live_udp_to_mavlink_controller.py \
  --runtime 20 \
  --packet-timeout 1.0 \
  --k-forward 100 \
  --k-yaw 120 \
  --max-x 120 \
  --max-r 120 \
  --yaw-deadband 0.04 \
  --forward-deadband 0.15 \
  --ema-alpha 0.35 \
  --max-delta-x-per-sec 240 \
  --max-delta-r-per-sec 260 \
  --invalid-decay-seconds 0.50 \
  --seq-jump-warning-threshold 10 \
  --log-csv logs/control_v2_clean_constant_on_arms_off.csv

Recommended armed command after arms-off is clean:

python scripts/06_live_udp_to_mavlink_controller.py \
  --runtime 12 \
  --packet-timeout 1.0 \
  --arm \
  --k-forward 100 \
  --k-yaw 120 \
  --max-x 120 \
  --max-r 120 \
  --yaw-deadband 0.04 \
  --forward-deadband 0.15 \
  --ema-alpha 0.35 \
  --max-delta-x-per-sec 240 \
  --max-delta-r-per-sec 260 \
  --invalid-decay-seconds 0.50 \
  --seq-jump-warning-threshold 10 \
  --log-csv logs/control_v2_clean_constant_on_armed.csv
4. Test with Clean Blink Video

After constant-ON tracking is stable, test the blink version:

BackOnly_Dynamic_Clean_01_BLINK.mp4

Control goal:

evaluate BIT_OFF,
evaluate held_observation,
evaluate INVALID_DECAY,
evaluate INVALID_STOP.

Expected:

more invalid/held states than constant-ON,
but smoother behavior than the original dynamic stress-test video.
5. Tune Controller Parameters

Controller parameters to tune:

k_forward
k_yaw
max_x
max_r
yaw_deadband
forward_deadband
ema_alpha
max_delta_x_per_sec
max_delta_r_per_sec
invalid_decay_seconds
packet_timeout

Suggested tuning order:

Start arms-off.
Check command logs.
Reduce saturation if target_x or target_r often hits max.
Increase deadband if small noise causes command jitter.
Increase ema_alpha if response is too slow.
Decrease ema_alpha if command is too jumpy.
Increase rate limits if motion is too weak.
Decrease rate limits if motion is too abrupt.
Only then run armed tests.
6. Move Toward Live Unity/Unreal Capture

The next major target is live image input.

Target future chain:

Unity or Unreal live render
→ OpenCV live sender
→ UDP observation
→ controller V2
→ Gazebo BlueROV2 motion

This will allow actual closed-loop behavior:

robot moves
→ camera image changes
→ error_x changes
→ controller reduces yaw command
→ distance error decreases

Success criteria:

error_x approaches zero,
yaw command decreases as the target becomes centered,
distance approaches desired distance,
forward command decreases near the target distance,
target loss triggers safe STOP or SEARCH behavior.
7. Future Controller Features

Potential future additions:

7.1 Search State

Add a low-speed yaw search when the target is lost for a controlled duration.

Example:

TRACK → INVALID_DECAY → INVALID_STOP → SEARCH

Search should only be enabled after STOP behavior is reliable.

7.2 Vertical Control

Current vertical command is fixed:

z = 500

Future vertical control may use:

error_norm[1]

Before adding vertical control:

verify vertical thruster mapping,
start with very small limits,
add vertical deadband,
test in isolation.
7.3 Binary UDP Packet

Current JSON packet is debugging-friendly. If latency or bandwidth becomes an issue, convert to a compact binary format.

Suggested binary fields:

magic
version
seq
valid
face_id
error_x
error_y
estimated_distance
distance_confidence
timestamp
crc
7.4 State Machine Refinement

Current controller already separates:

TRACK
INVALID_DECAY
INVALID_STOP
PACKET_TIMEOUT
NO_PACKET

Future states:

ALIGN_ONLY
SEARCH
STOP
FAILSAFE
8. Long-Term Goal

The long-term goal is full closed-loop target following:

live visual input
→ robust LED detection
→ face identification
→ smoothed controller
→ Gazebo/ArduSub motion
→ updated live visual input

The final controller should:

follow the target smoothly,
reduce image-center error,
maintain desired distance,
stop safely on target loss,
support multiple LED faces,
support future Unity and Unreal visual environments.
