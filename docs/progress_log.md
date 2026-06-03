# Progress Log

## Milestone 1 — MAVLink Connection and Basic Telemetry

Goal:

Verify that the Python control environment can connect to ArduSub through MAVLink.

Completed:

* established MAVLink connection through `udpin:127.0.0.1:14551`,
* received heartbeat,
* confirmed `target_system = 1`,
* confirmed `target_component = 0`,
* received telemetry messages such as:

  * `ATTITUDE`,
  * `VFR_HUD`,
  * `SYS_STATUS`,
  * `LOCAL_POSITION_NED`.

Result:

```text
Successful
```

---

## Milestone 2 — Manual Mode, Arm, Neutral Control and Disarm

Goal:

Verify that the Python controller can safely request MANUAL mode, arm the vehicle, send neutral `MANUAL_CONTROL`, and disarm.

Completed:

* MANUAL mode request,
* neutral command sending,
* arm command,
* disarm command,
* heartbeat-based armed/disarmed state checking.

Neutral command:

```text
x = 0
y = 0
z = 500
r = 0
```

Important implementation note:

The installed pymavlink version did not support `motors_armed_wait(timeout=...)`. The script was updated to check arm/disarm state through `HEARTBEAT.base_mode`.

Result:

```text
Successful
```

---

## Milestone 3 — Manual-Control Axis Mapping

Goal:

Identify how `MANUAL_CONTROL` x and r axes affect the BlueROV2 in Gazebo.

Completed:

```text
x = +250 → forward
x = -250 → backward
r = +250 → yaw right
r = -250 → yaw left
z = 500  → vertical neutral
```

Result:

```text
Successful
```

Control mapping decision:

```text
error_x > 0 → r positive
error_x < 0 → r negative

estimated_distance > desired_distance → x positive
estimated_distance < desired_distance → x negative
```

---

## Milestone 4 — UDP Observation Dry Run

Goal:

Receive a controller-ready UDP observation packet and compute a command without moving the vehicle.

Completed:

* UDP socket listening,
* JSON packet parsing,
* validation of:

  * `valid`,
  * `face_id`,
  * `pattern_accuracy`,
  * `distance_confidence`,
  * `error_norm`,
  * `estimated_distance`,
* command calculation.

Example input:

```text
valid = True
face_id = BACK
error_norm = [-0.0729, -0.0287]
estimated_distance = 2.888
```

Example command:

```text
cmd = (-44, 0, 500, -36)
```

Result:

```text
Successful
```

---

## Milestone 5 — First Safe UDP-to-MAVLink Controller

Script:

```text
scripts/05_udp_to_mavlink_controller_safe.py
```

Goal:

Connect UDP observation packets to MAVLink `MANUAL_CONTROL` safely.

Completed:

* UDP observation listening,
* observation validation,
* STOP on invalid packet,
* STOP on timeout,
* STOP on wrong face,
* STOP on low confidence,
* optional arming with `--arm`,
* STOP + DISARM on exit.

Result:

```text
Successful
```

Limitation:

Commands were direct and unsmoothed. When the observation changed from valid to invalid, commands jumped directly to STOP.

---

## Milestone 6 — Frame-Sequence and PNG-Based Vision Integration

Goal:

Move from a static observation packet to frame-sequence and image-derived UDP observations.

Completed with the vision repository:

```text
scripts/11_replay_back_observation_from_csv.py
scripts/12_live_back_png_sequence_sender.py
```

Control-side result:

* Linux received frame-varying UDP packets,
* `state=TRACK` stayed active while packets were valid,
* arms-off and armed tests completed safely.

Result:

```text
Successful
```

Limitation:

PNG and CSV replay were still offline inputs. They did not represent a live closed-loop camera.

---

## Milestone 7 — MP4 Video Sender Integration

Goal:

Use recorded Unity MP4 video as a dynamic visual input source.

Completed with the vision repository:

```text
scripts/13_live_back_video_sender.py
```

Control-side result:

* UDP video observations reached Linux,
* controller generated x/r commands,
* arm test completed with STOP + DISARM.

Issue observed:

The first video sender processed every third frame directly. Because the LED pattern blinks and the robot moves, some sampled frames were OFF or corrupted by motion/compression.

Result:

```text
Successful but not stable enough
```

---

## Milestone 8 — Vision Sender V2 and Better Observation Stability

Goal:

Improve observation continuity before control-side smoothing.

Completed with the vision repository:

```text
scripts/13_live_back_video_sender_v2.py
scripts/14_analyze_video_observation_log.py
scripts/15_render_video_detection_debug.py
```

Vision V2 improvements:

* every video frame processed,
* UDP still sent at 20 Hz,
* detection rate and send rate separated,
* best-pair candidate selection,
* reason-based hold:

  * `BIT_OFF`,
  * `LOW_CONFIDENCE`,
  * `CANDIDATE_COUNT_NOT_2`,
  * `PAIR_NOT_FOUND`,
* debug overlay video generation,
* observation log analysis.

Control-side impact:

* fewer unnecessary STOP transitions,
* longer `TRACK` periods,
* better observation continuity.

Result:

```text
Successful
```

---

## Milestone 9 — Controller V2 with Smoothing, Deadband and Invalid Decay

Script:

```text
scripts/06_live_udp_to_mavlink_controller.py
```

Goal:

Reduce command jumps and make control behavior smoother.

Completed:

* yaw deadband,
* forward deadband,
* EMA command smoothing,
* acceleration/rate limiting,
* invalid decay,
* hard STOP after long invalid,
* packet timeout STOP,
* CSV control logging,
* separate `validation` and `packet_reason` in printed logs,
* sequence-jump warning threshold.

Example behavior:

```text
TRACK          → smoothed command
INVALID_DECAY  → command gradually decreases
INVALID_STOP   → hard STOP
```

Result:

```text
Successful
```

---

## Milestone 10 — Gazebo / SITL / Thruster Verification

Script:

```text
scripts/07_manual_control_threshold_test.py
```

Goal:

Verify that Gazebo, ArduSub SITL, MAVLink and thruster topics are correctly connected.

Completed:

* clean process restart,
* Gazebo world verification,
* model verification,
* thruster topic verification,
* manual command threshold test.

Verified Gazebo model:

```text
bluerov2
```

Verified topics:

```text
/model/bluerov2/joint/thruster1_joint/cmd_thrust
/model/bluerov2/joint/thruster2_joint/cmd_thrust
/model/bluerov2/joint/thruster3_joint/cmd_thrust
/model/bluerov2/joint/thruster4_joint/cmd_thrust
/model/bluerov2/joint/thruster5_joint/cmd_thrust
/model/bluerov2/joint/thruster6_joint/cmd_thrust
/world/bluerov2_underwater/pose/info
```

Tested commands:

```text
x = 120
x = 180
r = 120
r = -120
```

Result:

```text
Successful
```

Conclusion:

Gazebo/SITL/MAVLink bridge works when started cleanly.

---

## Milestone 11 — Armed Video-to-Control Integration with Controller V2

Goal:

Validate the current full offline video-based integration chain in an armed Gazebo test.

Pipeline:

```text
OpenCV video sender V2
→ UDP observation packet
→ Linux controller V2
→ MAVLink MANUAL_CONTROL
→ Gazebo BlueROV2 motion
→ STOP + DISARM
```

Controller parameters:

```text
runtime = 12 s
k_forward = 100
k_yaw = 120
max_x = 120
max_r = 120
yaw_deadband = 0.04
forward_deadband = 0.15
ema_alpha = 0.35
max_delta_x_per_sec = 240
max_delta_r_per_sec = 260
invalid_decay_seconds = 0.50
```

Observed behavior:

* vehicle armed,
* UDP packets received,
* `TRACK` produced smoothed commands,
* Gazebo BlueROV2 moved,
* motion looked like forward movement with left yaw,
* this matched the logs because most observations had negative `error_x`,
* exit sent STOP,
* vehicle disarmed safely.

Result:

```text
Successful
```

Main conclusion:

The control side has now passed offline video-based armed integration.

---

## Current Status

The current control-side system can:

* connect to ArduSub through MAVLink,
* request MANUAL mode,
* arm/disarm safely,
* receive UDP observation packets,
* validate vision observations,
* compute x/r commands,
* smooth and limit commands,
* handle invalid observations with decay/STOP,
* move the Gazebo BlueROV2 in an armed test,
* stop and disarm safely.

Current limitation:

The input is still an offline recorded video. True closed-loop tracking is not yet tested.

Next target:

```text
Live Unity/Unreal render capture
→ OpenCV
→ UDP
→ controller V2
→ Gazebo motion
```

# Next Steps

## Current Status

The control side has passed the offline video-based integration milestone.

Validated chain:

```text
Unity recorded video
→ OpenCV video sender V2
→ UDP observation
→ Linux controller V2
→ MAVLink MANUAL_CONTROL
→ Gazebo BlueROV2 motion
→ STOP + DISARM
```

This proves that the communication and control pipeline works. However, it is still not true closed-loop tracking because the video input does not change when the Gazebo robot moves.

---

## 1. Preserve Current Stable Controller

The current main controller is:

```text
scripts/06_live_udp_to_mavlink_controller.py
```

Do not replace it immediately. Future experiments should either:

* add optional parameters,
* create a new script if behavior changes significantly,
* keep `05_udp_to_mavlink_controller_safe.py` as a simpler reference controller.

Current important controller features:

* yaw deadband,
* forward deadband,
* EMA smoothing,
* acceleration/rate limiting,
* invalid decay,
* hard STOP,
* CSV logging,
* heartbeat-based arm/disarm check.

---

## 2. Add Control Log Analysis

Create:

```text
scripts/08_analyze_control_log.py
```

Input example:

```text
logs/control_v2_armed_test_03.csv
```

The script should compute:

* total control samples,
* time spent in each state:

  * `TRACK`,
  * `INVALID_DECAY`,
  * `INVALID_STOP`,
  * `PACKET_TIMEOUT`,
  * `NO_PACKET`,
* average/max `target_x`,
* average/max `smooth_x`,
* average/max `target_r`,
* average/max `smooth_r`,
* command saturation ratio,
* number of STOP transitions,
* packet age statistics,
* held observation ratio,
* distance error range,
* yaw error range.

This will make controller tuning more systematic.

---

## 3. Test with Clean Constant-ON Video

The first dynamic video was useful as a stress test, but it included blink/OFF frames and temporary occlusion.

The next vision-side video should be:

```text
BackOnly_Dynamic_Clean_01_CONSTANT_ON.mp4
```

Control goal:

* evaluate pure tracking and controller smoothing without blink-induced target loss.

Expected controller behavior:

* more continuous `TRACK`,
* fewer `INVALID_DECAY` states,
* fewer `INVALID_STOP` states,
* smoother x/r commands.

Recommended arms-off command:

```bash
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
```

Recommended armed command after arms-off is clean:

```bash
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
```

---

## 4. Test with Clean Blink Video

After constant-ON tracking is stable, test the blink version:

```text
BackOnly_Dynamic_Clean_01_BLINK.mp4
```

Control goal:

* evaluate `BIT_OFF`,
* evaluate `held_observation`,
* evaluate `INVALID_DECAY`,
* evaluate `INVALID_STOP`.

Expected:

* more invalid/held states than constant-ON,
* but smoother behavior than the original dynamic stress-test video.

---

## 5. Tune Controller Parameters

Controller parameters to tune:

```text
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
```

Suggested tuning order:

1. Start arms-off.
2. Check command logs.
3. Reduce saturation if `target_x` or `target_r` often hits max.
4. Increase deadband if small noise causes command jitter.
5. Increase `ema_alpha` if response is too slow.
6. Decrease `ema_alpha` if command is too jumpy.
7. Increase rate limits if motion is too weak.
8. Decrease rate limits if motion is too abrupt.
9. Only then run armed tests.

---

## 6. Move Toward Live Unity/Unreal Capture

The next major target is live image input.

Target future chain:

```text
Unity or Unreal live render
→ OpenCV live sender
→ UDP observation
→ controller V2
→ Gazebo BlueROV2 motion
```

This will allow actual closed-loop behavior:

```text
robot moves
→ camera image changes
→ error_x changes
→ controller reduces yaw command
→ distance error decreases
```

Success criteria:

* `error_x` approaches zero,
* yaw command decreases as the target becomes centered,
* distance approaches desired distance,
* forward command decreases near the target distance,
* target loss triggers safe STOP or SEARCH behavior.

---

## 7. Future Controller Features

Potential future additions:

### 7.1 Search State

Add a low-speed yaw search when the target is lost for a controlled duration.

Example:

```text
TRACK → INVALID_DECAY → INVALID_STOP → SEARCH
```

Search should only be enabled after STOP behavior is reliable.

### 7.2 Vertical Control

Current vertical command is fixed:

```text
z = 500
```

Future vertical control may use:

```text
error_norm[1]
```

Before adding vertical control:

* verify vertical thruster mapping,
* start with very small limits,
* add vertical deadband,
* test in isolation.

### 7.3 Binary UDP Packet

Current JSON packet is debugging-friendly. If latency or bandwidth becomes an issue, convert to a compact binary format.

Suggested binary fields:

```text
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
```

### 7.4 State Machine Refinement

Current controller already separates:

```text
TRACK
INVALID_DECAY
INVALID_STOP
PACKET_TIMEOUT
NO_PACKET
```

Future states:

```text
ALIGN_ONLY
SEARCH
STOP
FAILSAFE
```

---

## 8. Long-Term Goal

The long-term goal is full closed-loop target following:

```text
live visual input
→ robust LED detection
→ face identification
→ smoothed controller
→ Gazebo/ArduSub motion
→ updated live visual input
```

The final controller should:

* follow the target smoothly,
* reduce image-center error,
* maintain desired distance,
* stop safely on target loss,
* support multiple LED faces,
* support future Unity and Unreal visual environments.


Controller V2 Arms-Off Test

The clean constant-ON video was streamed to the Linux controller using UDP. The controller was run without arming.

The controller correctly produced:

TRACK
INVALID_DECAY
INVALID_STOP
target command
smoothed command
MANUAL_CONTROL command

The yaw response matched the image error:

error_x positive → r positive
error_x negative → r negative

The forward response also matched the estimated distance:

estimated_distance > desired_distance → x positive
estimated_distance < desired_distance → x negative

The controller deadband worked correctly near the final centered region:

small error_x + distance near 3.0 → target=(0,0), cmd=(0,0,500,0)
Controller V2 Armed Test

A short armed Gazebo test was performed with the clean constant-ON video.

First armed test parameters:

k_forward = 90
k_yaw     = 110
max_x     = 100
max_r     = 100

Result:

- Robot moved in Gazebo.
- STOP and DISARM completed safely.
- Yaw response was visually too aggressive.

Reason:

The test still used an offline video. Therefore, the robot motion in Gazebo did not update the image error. When error_x stayed negative for a long period, the controller continued to command yaw, causing excessive rotation.

A lower-yaw armed test was then performed.

Second armed test parameters:

k_forward = 80
k_yaw     = 55
yaw_sign  = 1
max_x     = 90
max_r     = 45
yaw_deadband = 0.08
forward_deadband = 0.20
ema_alpha = 0.25
max_delta_x_per_sec = 160
max_delta_r_per_sec = 90

Result:

- Robot response was less aggressive.
- Yaw was still visually larger than ideal.
- This is expected because the input video is offline and the visual error cannot close.
- STOP and DISARM worked safely.

Conclusion:

The controller and vision-to-control pipeline are working. However, yaw tuning cannot be finalized using offline video because the image does not update in response to Gazebo motion.

Control-Side Update

The controller was extended with a yaw sign parameter:

--yaw-sign 1
--yaw-sign -1

Current mapping:

error_x > 0 → r positive
error_x < 0 → r negative

This mapping must be revalidated during live closed-loop testing. If the robot turns away from the target instead of reducing image error, the controller can be run with:

--yaw-sign -1

without changing the code.

Current Milestone

The following chain has now been validated:

Unity recorded constant-ON video
→ OpenCV sender V2
→ V2 LED pair selection
→ UDP observation packet
→ Linux controller V2
→ smoothing / deadband / invalid decay
→ MAVLink MANUAL_CONTROL
→ Gazebo BlueROV2 motion
→ STOP + DISARM

This is still offline video-based integration, not true closed-loop tracking.

Main Conclusion

The constant-ON dataset improved observation quality and reduced blink-related target loss. The V2 debug overlay confirmed that the selected LED pair is generally correct. Controller V2 successfully generated smooth forward/yaw commands and safely controlled the Gazebo vehicle.

The remaining limitation is that the input video is offline. Because the camera image does not update when the Gazebo robot moves, the controller cannot actually reduce the visual error. This makes yaw appear excessive in armed tests.

The next major step is live Unity or Unreal image capture.

Next Step

Move from recorded video to live visual input:

Unity live Game View / camera render
→ OpenCV live frame processing
→ UDP observation packet
→ Linux controller V2
→ Gazebo BlueROV2 motion

Initial live closed-loop controller parameters should be conservative:

k_forward = 70
k_yaw = 35
max_x = 70
max_r = 30
yaw_deadband = 0.10
forward_deadband = 0.25
ema_alpha = 0.20
max_delta_x_per_sec = 120
max_delta_r_per_sec = 60

The first live test should verify:

- Does error_x decrease when yaw command is applied?
- Is yaw_sign correct?
- Does estimated_distance move toward the desired distance?
- Does target loss trigger INVALID_DECAY / INVALID_STOP?
- Does STOP + DISARM always work?



## Progress Update — Unity Camera Pose Bridge Initial Validation

The Unity `CV_Test_Camera` was connected to a separate UDP pose receiver using port `5008`, while the leader robot kept its own receiver on port `5007`. This separation prevents port conflicts between the manually controlled leader object and the camera/follower visual pose.

The `GazeboDataReceiver` component was added to `CV_Test_Camera` with the following working test configuration:

```text
Listen Port              = 5008
Position Scale           = 1
Force Unity Start Pose   = false
Use Local Transform      = true
Keyboard Relative Mode   = true

Interpolation Delay      = 0.025
Max Buffer Samples       = 120

Use Smoothing            = true
Position Smooth Time     = 0.045
Rotation Smooth Speed    = 12
Position Deadband        = 0.0015
Rotation Deadband Deg    = 0.05
```

A Linux-to-Windows UDP test confirmed that Unity receives 9-float pose packets on port `5008` and that `CV_Test_Camera` can be moved through the receiver.

The current Linux pose bridge test uses:

```bash
--rate 90
--scale 0.40
--yaw-only
```

Initial observation:

```text
- UDP 5008 communication works.
- CV_Test_Camera moves from external pose packets.
- Yaw direction appears to be correctly matched between Gazebo and Unity.
- Position axes are not yet fully aligned.
```

Important note:

The next required calibration step is to fix the translation-axis mapping between Gazebo/MAVLink `LOCAL_POSITION_NED` and Unity camera motion. Yaw mapping should be preserved for now because the observed yaw direction is currently correct.


# Progress Log — Control / BlueROV2 LED Control Repository

## Gazebo → Unity Pose Bridge and Yaw Control Tests

The Gazebo / ArduSub / Unity / OpenCV / controller pipeline has been tested through multiple stages.

The validated integration chain is:

```text
Gazebo BlueROV2 / ArduSub
→ MAVLink pose output
→ 08_mavlink_pose_to_unity.py
→ UDP pose packet to Unity
→ CV_Test_Camera movement in Unity
→ Unity Game View live capture
→ OpenCV BACK LED observation
→ UDP observation packet to Linux controller
→ MAVLink MANUAL_CONTROL
→ Gazebo BlueROV2 yaw motion
```

## Unity Pose Receiver

Unity receives Gazebo pose data on:

```text
UDP port: 5008
```

The receiver script is attached to:

```text
CV_Test_Camera
```

A Windows firewall rule was added for UDP port 5008.

Important note:

```text
The Windows IP address can change depending on Wi-Fi / mobile hotspot / local network.
Always verify the current Windows IP using ipconfig before starting the pose bridge.
```

## Controller V2 Updates

`06_live_udp_to_mavlink_controller.py` was extended with held-observation scaling.

New / important options:

```text
--held-forward-scale
--held-yaw-scale
```

Held observations are stale visual measurements. The controller now scales command targets during held observations while keeping the raw `error_x` unchanged for logging and analysis.

Confirmed JSON field:

```text
held_observation
```

The sender uses `held_observation=True/False`, and the controller reads the same key.

## Distance Confidence Threshold Alignment

A critical issue was found and fixed:

```text
The sender and controller both have their own min-distance-confidence thresholds.
```

If the sender uses:

```text
--min-distance-confidence 0.40
```

but the controller remains at the old default:

```text
--min-distance-confidence 0.60
```

then the sender may transmit packets as valid, while the controller rejects them as `LOW_DISTANCE_CONFIDENCE`.

Current yaw-test baseline requires both sides to use:

```text
--min-distance-confidence 0.40
```

## Yaw-Only Pulse Test Script

A new yaw-only diagnostic script was added:

```text
scripts/09_yaw_only_pulse_test.py
```

Purpose:

```text
- Send only yaw MANUAL_CONTROL commands.
- Keep x=0, y=0, z=500 fixed.
- Avoid forward/backward movement during visual yaw diagnostics.
```

Safe diagnostic pulse found:

```text
r=8
duration=0.5 s
pause=3.0 s
```

Larger commands were too aggressive:

```text
r=80, duration=3.0 s:
  Too large. Target left the field of view.

r=20, duration=1.0 s:
  Still too large. Target moved close to the image edge.
```

## Yaw Controller Tuning

The yaw sign was validated:

```text
error_x > 0 → positive yaw command
error_x < 0 → negative yaw command
```

The high-authority yaw tests confirmed that the previous issue was not yaw sign, but excessive yaw authority and stale/lost observations.

Current preferred yaw baseline:

```text
k_yaw = 30
max_r = 12
yaw_sign = 1
yaw_deadband = 0.04
ema_alpha = 0.30
max_delta_r_per_sec = 80
held_yaw_scale = 0.4
min_distance_confidence = 0.40
```

Alternative held yaw scales were tested:

```text
held_yaw_scale = 0.2:
  Reduced stale-frame command strength, but became more passive and fragmented.

held_yaw_scale = 0.3:
  Gave acceptable first convergence, but later positive residual drift appeared in the test run.

held_yaw_scale = 0.4:
  Current preferred baseline because it provides stronger recovery and stable final centering.
```

D control is not added yet.

Reason:

```text
The current remaining issue is not mainly a classical yaw oscillation problem.
It is mostly caused by visual packet loss / held observations / target movement near the field of view boundary.
```

## Current Milestone

Yaw-only closed-loop control is now considered usable for the next phase.

Validated:

```text
- yaw sign
- safe yaw authority
- held observation scaling
- sender/controller confidence threshold alignment
- STOP and DISARM safety
```

Remaining before full forward tracking:

```text
- switch pose bridge from yaw-only to full pose
- verify forward motion changes Unity camera distance correctly
- verify pixel_distance increases when the follower moves forward
- verify estimated_distance decreases when moving forward
- add forward control with strong yaw gate and conservative gains
```

