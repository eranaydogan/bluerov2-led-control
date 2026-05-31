# Setup and Run

This document explains how to start the BlueROV2 LED-following control-side environment, verify the Gazebo/ArduSub connection, and run the current UDP-to-MAVLink controller tests.

The current validated pipeline is:

```text
OpenCV video sender V2
→ UDP observation packet
→ Linux controller V2
→ MAVLink MANUAL_CONTROL
→ ArduSub SITL
→ Gazebo BlueROV2 motion
→ STOP + DISARM safety
```

The controller repository runs on Linux. The vision repository usually runs on Windows.

---

## 1. Create Python Environment

```bash
cd ~/bluerov2-led-control

python3 -m venv .venv
source .venv/bin/activate

pip install -r requirements.txt
```

Verify `pymavlink`:

```bash
python -c "from pymavlink import mavutil; print('pymavlink OK')"
```

Expected:

```text
pymavlink OK
```

---

## 2. Clean Old Processes Before Starting

Before every serious test, stop old Gazebo/SITL/MAVProxy processes.

```bash
pkill -f sim_vehicle.py
pkill -f mavproxy.py
pkill -f ardusub
pkill -f gz
pkill -f gazebo
pkill -f ruby
```

Check that old ports are clean:

```bash
ss -lntup | grep -E "14550|14551|5760|5501|9002|9003" || echo "ilgili portlarda çalışan süreç yok"
```

Expected if clean:

```text
ilgili portlarda çalışan süreç yok
```

This step is important. A previous issue where Gazebo did not visually move was caused by an unclean or incorrect Gazebo/SITL instance. After restarting cleanly, the thruster topics and Gazebo motion worked correctly.

---

## 3. Gazebo Environment Variables

Before starting Gazebo, verify the required paths:

```bash
echo "ROS_DISTRO=$ROS_DISTRO"
echo "GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH"
echo "GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
```

Expected examples:

```text
ROS_DISTRO=jazzy
GZ_SIM_SYSTEM_PLUGIN_PATH=/home/eren/ardupilot_gazebo/build:
GZ_SIM_RESOURCE_PATH=/home/eren/colcon_ws/src/bluerov2_gz/models:/home/eren/colcon_ws/src/bluerov2_gz/worlds:/home/eren/ardupilot_gazebo/models:/home/eren/ardupilot_gazebo/worlds:/opt/ros/jazzy/share
```

If needed, export them manually:

```bash
export GZ_IP=127.0.0.1
export IGN_IP=127.0.0.1

export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH

export GZ_SIM_RESOURCE_PATH=$HOME/colcon_ws/src/bluerov2_gz/models:$HOME/colcon_ws/src/bluerov2_gz/worlds:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
```

The warning below may appear in Gazebo:

```text
Exception sending a multicast message: Network is unreachable
```

This warning is not necessarily fatal. If the BlueROV2 model exists, thruster topics exist, thrust data flows, and Gazebo motion is visible, the simulation connection is working.

---

## 4. Start Gazebo

Start Gazebo in a dedicated terminal:

```bash
gz sim -v 4 -r bluerov2_underwater.world
```

Verify that the correct world and model are loaded:

```bash
gz model --list
```

Expected:

```text
Requesting state for world [bluerov2_underwater]...

Available models:
    - sand_heightmap
    - bluerov2
    - axes
```

Verify thruster and pose topics:

```bash
gz topic -l | grep -i thruster
```

Expected topics include:

```text
/model/bluerov2/joint/thruster1_joint/cmd_thrust
/model/bluerov2/joint/thruster2_joint/cmd_thrust
/model/bluerov2/joint/thruster3_joint/cmd_thrust
/model/bluerov2/joint/thruster4_joint/cmd_thrust
/model/bluerov2/joint/thruster5_joint/cmd_thrust
/model/bluerov2/joint/thruster6_joint/cmd_thrust
```

Pose topic:

```bash
gz topic -l | grep pose
```

Expected:

```text
/world/bluerov2_underwater/pose/info
/world/bluerov2_underwater/dynamic_pose/info
```

Important:

Use the correct world name:

```bash
gz topic -e -t /world/bluerov2_underwater/pose/info
```

Do not use:

```bash
/world/default/pose/info
```

because the active world is `bluerov2_underwater`.

---

## 5. Start ArduSub SITL

Start SITL in another terminal:

```bash
. ~/.profile
cd ~/ardupilot

sim_vehicle.py -L RATBeach -v ArduSub -f vectored --model=JSON \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14551 \
  --console
```

Port usage:

```text
14550 → QGroundControl or external monitoring
14551 → Python controller
```

The controller connects to:

```text
udpin:127.0.0.1:14551
```

---

## 6. Verify Gazebo–SITL–MAVLink Connection

Before running the vision-controller test, verify that direct MANUAL_CONTROL commands move the Gazebo model.

Open a monitoring terminal:

```bash
gz topic -e -t /model/bluerov2/joint/thruster1_joint/cmd_thrust
```

Then run the threshold test:

```bash
cd ~/bluerov2-led-control
source .venv/bin/activate

python scripts/07_manual_control_threshold_test.py
```

Expected:

* `thruster1_joint/cmd_thrust` receives data,
* the Gazebo BlueROV2 moves,
* `LOCAL_POSITION_NED` and/or `VFR_HUD` values change,
* yaw tests rotate the vehicle.

This test confirms that the issue is not Gazebo/SITL/MAVLink before running the vision pipeline.

---

## 7. Run Basic MAVLink and Manual-Control Tests

### 7.1 MAVLink connection check

```bash
cd ~/bluerov2-led-control
source .venv/bin/activate

python scripts/01_mavlink_connection_check.py
```

Expected:

* heartbeat received,
* target system and component printed,
* telemetry messages received.

### 7.2 Manual STOP test

```bash
python scripts/02_manual_control_stop_test.py
```

Expected:

* MANUAL mode requested,
* vehicle arms,
* neutral `MANUAL_CONTROL` sent,
* vehicle disarms safely.

### 7.3 Axis mapping test

```bash
python scripts/03_manual_control_axis_test.py
```

Expected mapping:

```text
x = +250 → forward in vehicle heading direction
x = -250 → backward
r = +250 → yaw right
r = -250 → yaw left
z = 500  → vertical neutral
```

---

## 8. Old Safe Controller

The first safe controller is still available:

```text
scripts/05_udp_to_mavlink_controller_safe.py
```

It can be used for simple UDP-to-MAVLink tests.

Arms-off:

```bash
python scripts/05_udp_to_mavlink_controller_safe.py \
  --runtime 20 \
  --packet-timeout 1.0
```

Armed:

```bash
python scripts/05_udp_to_mavlink_controller_safe.py \
  --runtime 20 \
  --packet-timeout 1.0 \
  --arm
```

This controller sends direct commands without advanced smoothing. It is kept for reference and fallback testing.

---

## 9. Current Controller V2

The current main controller is:

```text
scripts/06_live_udp_to_mavlink_controller.py
```

It adds:

* yaw deadband,
* forward deadband,
* EMA command smoothing,
* command rate limiting,
* invalid decay,
* hard STOP on timeout or long invalid periods,
* CSV logging,
* separate validation reason and packet reason in logs.

Control mapping:

```text
error_x > 0 → r positive
error_x < 0 → r negative

estimated_distance > desired_distance → x positive
estimated_distance < desired_distance → x negative

z = 500 → vertical neutral
```

---

## 10. Run Controller V2 Without Arming

Start the Linux controller:

```bash
cd ~/bluerov2-led-control
source .venv/bin/activate

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
  --log-csv logs/control_v2_arms_off_test.csv
```

When the controller prints:

```text
Listening UDP on 0.0.0.0:5005
```

start the Windows OpenCV video sender V2.

Example Windows command:

```powershell
cd C:\Dev\PythonProjects\OpenCV\png_tracker
.\.venv\Scripts\Activate.ps1

python .\scripts\13_live_back_video_sender_v2.py `
  --video .\datasets\videos\BackOnly_Dynamic_Test_01.mp4 `
  --dataset BackOnly_Dynamic_Test_01_v2_controller_test `
  --ip 192.168.137.228 `
  --port 5005 `
  --rate 20 `
  --loop `
  --allow-more-than-two-candidates `
  --pair-strategy best
```

Expected controller states:

```text
TRACK
INVALID_DECAY
INVALID_STOP
PACKET_TIMEOUT
```

Expected behavior:

* `TRACK` produces smoothed x/r commands,
* `INVALID_DECAY` gradually reduces commands,
* `INVALID_STOP` sends hard STOP,
* no vehicle movement occurs because `--arm` is not used.

---

## 11. Run Controller V2 With Arming

Only run armed tests after:

* Gazebo model is visible,
* thruster topics exist,
* SITL is running,
* threshold test confirms motion,
* arms-off controller test is clean.

Linux:

```bash
cd ~/bluerov2-led-control
source .venv/bin/activate

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
  --log-csv logs/control_v2_armed_test.csv
```

Windows sender:

```powershell
python .\scripts\13_live_back_video_sender_v2.py `
  --video .\datasets\videos\BackOnly_Dynamic_Test_01.mp4 `
  --dataset BackOnly_Dynamic_Test_01_v2_armed_test `
  --ip 192.168.137.228 `
  --port 5005 `
  --rate 20 `
  --loop `
  --allow-more-than-two-candidates `
  --pair-strategy best
```

Expected:

* vehicle arms,
* `TRACK` commands produce visible Gazebo motion,
* `INVALID_DECAY` smoothly reduces command,
* `INVALID_STOP` sends neutral command,
* exit sends STOP,
* vehicle disarms safely.

In the first successful armed V2 video test, the vehicle generally moved forward while yawing left. This was expected because most video observations had negative `error_x`, so the controller generated negative `r` commands.

---

## 12. Safety Behavior

The controller sends STOP or neutral command when:

* no UDP packet has been received,
* packet timeout occurs,
* packet is invalid,
* `face_id` is not `BACK`,
* pattern confidence is too low,
* distance confidence is too low,
* error or distance fields are missing/non-finite,
* the test exits,
* the user interrupts the program.

STOP command:

```text
x = 0
y = 0
z = 500
r = 0
```

If the script armed the vehicle, it disarms the vehicle before exit.

---

## 13. Useful Monitoring Commands

Monitor a thruster command:

```bash
gz topic -e -t /model/bluerov2/joint/thruster1_joint/cmd_thrust
```

Monitor Gazebo pose:

```bash
gz topic -e -t /world/bluerov2_underwater/pose/info
```

List models:

```bash
gz model --list
```

List BlueROV2 topics:

```bash
gz topic -l | grep -i blue
```

List thruster topics:

```bash
gz topic -l | grep -i thruster
```

Check MAVLink/SITL ports:

```bash
ss -lntup | grep -E "14550|14551|5760|5501"
```

---

## 14. Notes

* The current vision input is still an offline recorded video.
* Offline video tests verify the full communication and control chain, but not true closed-loop tracking.
* True closed-loop behavior requires live Unity/Unreal render capture.
* In offline video tests, if most `error_x` values are negative, the robot will keep yawing left because the image feedback does not change in response to robot motion.

