# Test Log

## 01 MAVLink connection check

Result: successful.

Confirmed:

- MAVLink heartbeat received
- target_system = 1
- target_component = 0
- ATTITUDE, VFR_HUD, SYS_STATUS, LOCAL_POSITION_NED messages received

## 02 Manual control stop test

Result: successful.

Confirmed:

- MANUAL mode requested
- vehicle armed
- neutral MANUAL_CONTROL sent
- vehicle disarmed safely

## 03 Manual control axis test

Result: successful.

Observed mapping:

- x = +250 -> forward in vehicle heading direction
- x = -250 -> backward
- r = +250 -> yaw right
- r = -250 -> yaw left

## 04 UDP dry-run controller

Result: successful.

Observed packet:

- valid=True
- face=BACK
- pattern_accuracy=1.0
- distance_confidence=1.0
- error_norm=[-0.0729, -0.0287]
- estimated_distance=2.888

Computed command:

- cmd=(-44,0,500,-36)

## 05 UDP to MAVLink safe controller

Result: successful.

Arms-off test:

- state=TRACK
- packets received continuously
- command computed continuously
- vehicle not armed

Armed test:

- vehicle armed successfully
- state=TRACK stayed active while UDP packets were fresh
- command sent continuously
- vehicle yawed left as expected because static packet produced r=-36
- STOP sent before exit
- vehicle disarmed safely

Note:

The armed test used a static observation packet. Therefore the command stayed constant and the robot kept yawing left. This is expected. Live closed-loop testing requires real-time vision packets.
