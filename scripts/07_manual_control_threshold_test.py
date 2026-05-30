import time
from pymavlink import mavutil

MAVLINK = "udpin:127.0.0.1:14551"
RATE_HZ = 20
Z_NEUTRAL = 500


def send_manual(master, x, y, z, r):
    master.mav.manual_control_send(
        master.target_system,
        int(x),
        int(y),
        int(z),
        int(r),
        0,
    )


def send_for(master, x, y, z, r, duration):
    n = int(duration * RATE_HZ)
    for _ in range(n):
        send_manual(master, x, y, z, r)
        time.sleep(1.0 / RATE_HZ)


def read_status(master, duration=1.0):
    end = time.time() + duration
    while time.time() < end:
        msg = master.recv_match(
            type=["VFR_HUD", "LOCAL_POSITION_NED", "ATTITUDE", "HEARTBEAT"],
            blocking=True,
            timeout=0.2,
        )
        if msg is None:
            continue

        t = msg.get_type()

        if t == "VFR_HUD":
            print(
                f"  VFR_HUD gs={msg.groundspeed:.3f} "
                f"thr={msg.throttle} alt={msg.alt:.3f} climb={msg.climb:.3f}"
            )

        elif t == "LOCAL_POSITION_NED":
            print(
                f"  POS x={msg.x:.3f} y={msg.y:.3f} z={msg.z:.3f} "
                f"vx={msg.vx:.3f} vy={msg.vy:.3f} vz={msg.vz:.3f}"
            )

        elif t == "ATTITUDE":
            print(
                f"  ATT roll={msg.roll:.3f} pitch={msg.pitch:.3f} yaw={msg.yaw:.3f}"
            )

        elif t == "HEARTBEAT":
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            print(f"  HB armed={armed} mode={msg.custom_mode}")


def wait_armed(master, expected=True, timeout=10):
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if msg is None:
            continue
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f"HEARTBEAT armed={armed} base_mode={msg.base_mode} custom_mode={msg.custom_mode}")
        if armed == expected:
            return True
    return False


def arm(master):
    print("ARM...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0,
    )
    if not wait_armed(master, True):
        raise RuntimeError("Arm failed")


def disarm(master):
    print("DISARM...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0,
    )
    wait_armed(master, False)


def set_manual(master):
    modes = master.mode_mapping()
    mode_id = modes["MANUAL"]
    print(f"Set MANUAL mode_id={mode_id}")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )
    time.sleep(1.0)


def stop(master, duration=1.5):
    print("STOP")
    send_for(master, 0, 0, Z_NEUTRAL, 0, duration)


def test_case(master, name, x=0, r=0, duration=3.0):
    print("")
    print(f"=== {name}: x={x}, r={r}, duration={duration}s ===")
    read_status(master, 0.5)
    send_for(master, x, 0, Z_NEUTRAL, r, duration)
    read_status(master, 1.5)
    stop(master, 1.5)
    read_status(master, 0.8)


def main():
    master = mavutil.mavlink_connection(MAVLINK)

    print("Waiting heartbeat...")
    master.wait_heartbeat()
    print(f"Heartbeat OK target_system={master.target_system} target_component={master.target_component}")

    set_manual(master)
    stop(master, 1.5)
    arm(master)
    stop(master, 1.5)

    # Threshold sweep
    test_case(master, "FORWARD_X_120", x=120, r=0, duration=3.0)
    test_case(master, "FORWARD_X_180", x=180, r=0, duration=3.0)
    test_case(master, "YAW_R_120", x=0, r=120, duration=3.0)
    test_case(master, "YAW_R_NEG_120", x=0, r=-120, duration=3.0)

    stop(master, 1.5)
    disarm(master)
    stop(master, 1.0)

    print("Threshold test finished.")


if __name__ == "__main__":
    main()
