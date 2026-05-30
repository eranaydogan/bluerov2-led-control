# Setup and Run

## 1. Create Python environment

```bash
cd ~/bluerov2-led-control
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## 2. Start Gazebo

```bash
source ~/.bashrc
gz sim -v 4 -r bluerov2_underwater.world
```

## 3. Start ArduSub SITL

```bash
. ~/.profile
cd ~/ardupilot

sim_vehicle.py -L RATBeach -v ArduSub -f vectored --model=JSON \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14551 \
  --console
```

## 4. Run safe controller without arming

```bash
cd ~/bluerov2-led-control
source .venv/bin/activate

python scripts/05_udp_to_mavlink_controller_safe.py \
  --runtime 20 \
  --packet-timeout 1.0
```

## 5. Send UDP observation packet from Windows

```bash
python .\scripts\09_udp_send_observation.py --dataset BackOnly_Test_04 --frame 120 --ip <LINUX_IP> --port 5005 --count 1000 --rate 20
```

## 6. Run safe controller with arming

```bash
cd ~/bluerov2-led-control
source .venv/bin/activate

python scripts/05_udp_to_mavlink_controller_safe.py \
  --runtime 20 \
  --packet-timeout 1.0 \
  --arm
```

## Safety behavior
The controller sends STOP when:

* no UDP packet is received,
* packet timeout occurs,
* packet is invalid,
* face_id is not BACK,
* pattern confidence is too low.

At exit, the controller sends STOP and disarms if it armed the vehicle.
