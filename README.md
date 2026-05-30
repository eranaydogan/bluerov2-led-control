# BlueROV2 LED-Based Control Side

This repository contains the Linux-side control scripts for the BlueROV2 LED-based visual tracking project.

The control side receives visual observation packets from the Windows/OpenCV side over UDP and converts them into MAVLink MANUAL_CONTROL commands for ArduSub SITL.

## Current validated pipeline

Windows/OpenCV observation packet:

- valid
- face_id
- pattern_accuracy
- distance_confidence
- error_norm
- estimated_distance

Linux control conversion:

- error_norm[0] -> yaw command `r`
- estimated_distance - desired_distance -> forward/backward command `x`
- vertical control is currently disabled, `z=500`

Control output:

- MAVLink MANUAL_CONTROL
- ArduSub SITL
- Gazebo Harmonic BlueROV2 simulation

## Important design decision

The project uses MAVLink MANUAL_CONTROL.

RC override and MAVProxy `rc` commands are not used as the standard control method.

## Script order

1. `01_mavlink_connection_check.py`
2. `02_manual_control_stop_test.py`
3. `03_manual_control_axis_test.py`
4. `04_udp_dry_run_controller.py`
5. `05_udp_to_mavlink_controller_safe.py`

## Validated axis mapping

From simulation tests:

- `x > 0` moves the robot forward in its current heading direction
- `x < 0` moves the robot backward
- `r > 0` yaws right
- `r < 0` yaws left
- `z = 500` is neutral vertical/heave command

## Latest integration status

The safe UDP-to-MAVLink controller was tested successfully.

The controller:

- receives UDP packets continuously,
- computes `cmd=(-44,0,500,-36)` for the static BackOnly_Test_04 packet,
- sends MAVLink MANUAL_CONTROL continuously,
- arms successfully when `--arm` is used,
- keeps `state=TRACK` while packets are fresh,
- sends STOP on exit,
- disarms safely.

This is a static-packet integration test, not yet a live closed-loop tracking test.
