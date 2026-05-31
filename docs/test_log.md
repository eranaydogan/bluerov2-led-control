# Test Log

This document records the main control-side tests performed for the BlueROV2 LED-based tracking and control pipeline.

---

## 01 — MAVLink Connection Check

Script:

```text
scripts/01_mavlink_connection_check.py
```

Result:

```text
Successful
```

Confirmed:

* MAVLink heartbeat received,
* `target_system = 1`,
* `target_component = 0`,
* `ATTITUDE` messages received,
* `VFR_HUD` messages received,
* `SYS_STATUS` messages received,
* `LOCAL_POSITION_NED` messages received.

Conclusion:

The Python control environment can connect to ArduSub through MAVLink.

---

## 02 — Manual Control STOP Test

Script:

```text
scripts/02_manual_control_stop_test.py
```

Result:

```text
Successful
```

Confirmed:

* MANUAL mode requested,
* vehicle armed,
* neutral `MANUAL_CONTROL` sent,
* vehicle disarmed safely.

Neutral command:

```text
x = 0
y = 0
z = 500
r = 0
```

Note:

The initial pymavlink helper calls using `timeout=` were not compatible with the installed pymavlink version. The script was adjusted to check armed/disarmed state using `HEARTBEAT.base_mode`.

---

## 03 — Manual Control Axis Test

Script:

```text
scripts/03_manual_control_axis_test.py
```

Result:

```text
Successful
```

Observed mapping:

```text
x = +250 → forward in vehicle heading direction
x = -250 → backward
r = +250 → yaw right
r = -250 → yaw left
z = 500  → vertical neutral
```

Conclusion:

The controller should map image and distance errors as follows:

```text
error_x > 0 → r positive
error_x < 0 → r negative

estimated_distance > desired_distance → x positive
estimated_distance < desired_distance → x negative
```

---

## 04 — UDP Dry-Run Controller

Script:

```text
scripts/04_udp_dry_run_controller.py
```

Result:

```text
Successful
```

Observed packet:

```text
valid = True
face_id = BACK
pattern_accuracy = 1.0
distance_confidence = 1.0
error_norm = [-0.0729, -0.0287]
estimated_distance = 2.888
```

Computed command:

```text
cmd = (-44, 0, 500, -36)
```

Interpretation:

```text
estimated_distance = 2.888 < desired_distance = 3.0
→ target is slightly close
→ x negative
→ move backward slightly

error_x = -0.0729
→ target is left of image center
→ r negative
→ yaw left
```

Conclusion:

The control-side UDP packet parser and command calculation logic worked correctly in dry-run mode.

---

## 05 — UDP to MAVLink Safe Controller

Script:

```text
scripts/05_udp_to_mavlink_controller_safe.py
```

Result:

```text
Successful
```

### Arms-off test

Confirmed:

* UDP packets received,
* `state=TRACK` while packets were valid and fresh,
* command computed continuously,
* vehicle was not armed.

### Armed test with static packet

Confirmed:

* vehicle armed successfully,
* `state=TRACK` while UDP packets were fresh,
* command sent continuously,
* robot yawed left as expected,
* STOP sent before exit,
* vehicle disarmed safely.

Important note:

The armed test used a static observation packet. Therefore, the command stayed almost constant and the robot kept yawing left. This was expected and was not a controller failure. A real tracking test requires frame-varying or live observations.

---

## 06 — CSV Replay UDP Observation Test

Vision-side script:

```text
scripts/11_replay_back_observation_from_csv.py
```

Control-side script:

```text
scripts/05_udp_to_mavlink_controller_safe.py
```

Result:

```text
Successful
```

Confirmed:

* frame-sequence observations were sent from Windows to Linux,
* UDP sequence increased,
* packet age stayed low,
* Linux controller stayed in `TRACK` when packets were valid,
* packet timeout did not occur during normal streaming.

Conclusion:

The system moved from static single-packet testing to frame-sequence UDP observation streaming.

---

## 07 — PNG Sequence OpenCV Sender Test

Vision-side script:

```text
scripts/12_live_back_png_sequence_sender.py
```

Control-side script:

```text
scripts/05_udp_to_mavlink_controller_safe.py
```

Result:

```text
Successful
```

Confirmed:

* PNG frames were processed directly by OpenCV,
* BACK LED pair was detected,
* `error_norm` and `estimated_distance` were generated,
* UDP observation packets reached Linux,
* Linux controller produced correct x/r commands,
* arms-off and armed tests completed safely.

Conclusion:

The CSV dependency was removed for this stage. Observation packets were generated from actual image frames.

---

## 08 — MP4 Video Sender Test

Vision-side script:

```text
scripts/13_live_back_video_sender.py
```

Control-side script:

```text
scripts/05_udp_to_mavlink_controller_safe.py
```

Result:

```text
Successful, but detection was not yet stable enough
```

Confirmed:

* MP4 video opened correctly,
* 60 FPS video was downsampled to 20 Hz observation stream,
* UDP packets reached Linux,
* controller produced correct x/r commands,
* armed test completed with STOP + DISARM.

Observed issue:

The first video sender processed every third frame directly. This sometimes sampled LED OFF frames or motion-corrupted frames.

Observed states included:

```text
OK
BIT_OFF
LOW_CONFIDENCE
CANDIDATE_COUNT_NOT_2
PAIR_NOT_FOUND
HELD_DURING_BIT_OFF
```

Conclusion:

The video sender worked, but tracking was too discontinuous. A more stable V2 sender was needed.

---

## 09 — Video Observation Log Analysis

Vision-side script:

```text
scripts/14_analyze_video_observation_log.py
```

Result:

```text
Successful
```

The first video analysis showed that invalid packets were frequent.

Main invalid reasons:

```text
BIT_OFF
CANDIDATE_COUNT_NOT_2
LOW_CONFIDENCE
PAIR_NOT_FOUND
```

Conclusion:

The log analysis showed two necessary improvements:

```text
1. reason-based hold
2. candidate_count > 2 best-pair selection
```

---

## 10 — Debug Overlay Video Test

Vision-side script:

```text
scripts/15_render_video_detection_debug.py
```

Result:

```text
Successful
```

Confirmed visually:

* selected green boxes usually corresponded to the correct BACK LED pair,
* candidate_count 3 or 4 cases could often still select the correct pair,
* fish occlusion did not generally cause a wrong LED pair selection,
* LED OFF frames correctly produced invalid states,
* some ON frames were still missed due to motion, blink, or MP4 compression,
* midpoint moved correctly during left/right target motion.

Conclusion:

Using more than two candidates was acceptable for the current video, but a better pair-scoring strategy was still needed.

---

## 11 — Video Sender V2 Offline Analysis

Vision-side script:

```text
scripts/13_live_back_video_sender_v2.py
```

Result:

```text
Successful
```

V2 improvements:

```text
- every video frame is processed,
- UDP is still sent at 20 Hz,
- detection rate and send rate are separated,
- reason-based hold added,
- best-pair selection added,
- video_observation_log_v2.csv generated.
```

V2 achieved a much better valid observation ratio than the first video sender.

Conclusion:

The V2 video sender became the current recommended vision-side sender for offline video integration tests.

---

## 12 — Controller V2 Arms-Off Test

Control-side script:

```text
scripts/06_live_udp_to_mavlink_controller.py
```

Vision-side script:

```text
scripts/13_live_back_video_sender_v2.py
```

Result:

```text
Successful
```

Confirmed:

* V2 UDP packets received,
* `TRACK` state appeared for valid or held observations,
* `INVALID_DECAY` appeared for short invalid periods,
* `INVALID_STOP` appeared for long invalid periods,
* target command and smooth command were separated,
* command smoothing worked,
* command did not jump instantly between full command and STOP,
* vehicle was not armed.

Conclusion:

Controller V2 behaved correctly in arms-off mode.

---

## 13 — Controller V2 Sequence-Jump and Logging Patch

Issue:

The first V2 controller printed many sequence-jump warnings because it intentionally used the latest UDP packet and skipped older queued packets.

Fix:

```text
--seq-jump-warning-threshold
```

was added.

The controller also started printing both:

```text
validation=<controller validation result>
packet_reason=<vision packet reason>
```

Example:

```text
validation=OK packet_reason=HELD_DURING_BIT_OFF
validation=PACKET_INVALID packet_reason=BIT_OFF
```

Result:

```text
Successful
```

Conclusion:

The logs became more readable and easier to debug.

---

## 14 — Gazebo / SITL / Thruster Connection Verification

Script:

```text
scripts/07_manual_control_threshold_test.py
```

Result:

```text
Successful
```

Reason for test:

An earlier armed V2 controller test did not show visible Gazebo motion. The issue was suspected to be a stale or incorrectly started Gazebo/SITL instance.

Clean restart procedure was performed, then the following were verified:

```text
gz model --list
→ bluerov2 exists

gz topic -l | grep -i thruster
→ thruster cmd_thrust topics exist

gz topic -e -t /model/bluerov2/joint/thruster1_joint/cmd_thrust
→ thrust data flows during manual control test
```

Threshold test commands:

```text
x = 120
x = 180
r = 120
r = -120
```

Confirmed:

* thruster topic data flowed,
* Gazebo BlueROV2 moved,
* MAVLink/SITL/Gazebo bridge was correct.

Conclusion:

The previous “no movement” issue was not a controller-code issue. It was likely caused by an unclean or wrong simulation startup.

---

## 15 — Controller V2 Armed Video Integration Test

Control-side script:

```text
scripts/06_live_udp_to_mavlink_controller.py
```

Vision-side script:

```text
scripts/13_live_back_video_sender_v2.py
```

Result:

```text
Successful
```

Test parameters:

```text
runtime = 12 s
arm = true
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
* valid/held observations generated smoothed commands,
* robot moved in Gazebo,
* the motion looked like forward movement with left yaw,
* this matched the log because most observations had negative `error_x`,
* invalid periods triggered decay or STOP,
* test exit sent STOP,
* vehicle disarmed safely.

Example command pattern:

```text
cmd=(110,0,500,-62)
cmd=(120,0,500,-62)
cmd=(85,0,500,-67)
cmd=(48,0,500,-80)
cmd=(117,0,500,-83)
cmd=(120,0,500,-70)
```

Interpretation:

```text
x positive → forward movement
r negative → yaw left
```

Conclusion:

The full offline video-based integration pipeline was successfully validated:

```text
OpenCV video sender V2
→ UDP observation
→ controller V2 smoothing/deadband
→ MAVLink MANUAL_CONTROL
→ Gazebo BlueROV2 motion
→ STOP + DISARM
```

---

## Current Overall Conclusion

The control side is now validated up to offline video-based armed integration.

Confirmed:

* MAVLink connection works,
* MANUAL_CONTROL mapping works,
* Gazebo thruster topics receive data,
* Gazebo BlueROV2 moves,
* UDP observation packets are received,
* controller V2 smooths and limits commands,
* invalid observations are handled safely,
* armed test ends with STOP + DISARM.

Current limitation:

This is not yet true closed-loop tracking because the vision input is a pre-recorded video. The video does not change when the Gazebo robot moves.

Next major target:

```text
Live Unity/Unreal render capture
→ OpenCV
→ UDP
→ controller V2
→ Gazebo
```

