import argparse
import time
from pymavlink import mavutil


DEFAULT_MAVLINK = "udpin:127.0.0.1:14551"
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


def stop(master, duration=1.5):
    print(f"STOP for {duration:.1f}s")
    send_for(master, 0, 0, Z_NEUTRAL, 0, duration)


def wait_armed(master, expected=True, timeout=10):
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
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

    if "MANUAL" not in modes:
        raise RuntimeError(f"MANUAL mode not found. Available modes: {list(modes.keys())}")

    mode_id = modes["MANUAL"]
    print(f"Set MANUAL mode_id={mode_id}")

    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )

    time.sleep(1.0)


def pulse(master, name, x, duration, pause):
    print("")
    print("=" * 70)
    print(f"{name}: x={x}, y=0, z={Z_NEUTRAL}, r=0, duration={duration:.1f}s")
    print("=" * 70)

    send_for(master, x, 0, Z_NEUTRAL, 0, duration)
    stop(master, pause)


def main():
    parser = argparse.ArgumentParser(
        description="Forward-only MAVLink MANUAL_CONTROL pulse test for BlueROV2 / ArduSub."
    )

    parser.add_argument("--mavlink", default=DEFAULT_MAVLINK)
    parser.add_argument("--x", type=int, default=8)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--pause", type=float, default=2.5)
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--start-negative", action="store_true")
    parser.add_argument("--no-disarm", action="store_true")

    args = parser.parse_args()

    x = max(1, min(abs(args.x), 1000))

    print("=== Forward Only Pulse Test ===")
    print(f"MAVLink   : {args.mavlink}")
    print(f"x command : +/-{x}")
    print(f"duration  : {args.duration:.1f}s")
    print(f"pause     : {args.pause:.1f}s")
    print(f"cycles    : {args.cycles}")
    print("Yaw/side/vertical commands are disabled: y=0, z=500, r=0")
    print("")

    master = mavutil.mavlink_connection(args.mavlink)

    print("Waiting heartbeat...")
    master.wait_heartbeat()
    print(
        f"Heartbeat OK target_system={master.target_system} "
        f"target_component={master.target_component}"
    )

    set_manual(master)

    print("Sending neutral command before arm...")
    stop(master, 1.5)

    arm(master)

    print("Sending neutral command after arm...")
    stop(master, 1.5)

    try:
        for i in range(args.cycles):
            print("")
            print(f"######## Cycle {i + 1}/{args.cycles} ########")

            if args.start_negative:
                pulse(master, "BACKWARD_OR_NEGATIVE_X", -x, args.duration, args.pause)
                pulse(master, "FORWARD_OR_POSITIVE_X", +x, args.duration, args.pause)
            else:
                pulse(master, "FORWARD_OR_POSITIVE_X", +x, args.duration, args.pause)
                pulse(master, "BACKWARD_OR_NEGATIVE_X", -x, args.duration, args.pause)

    except KeyboardInterrupt:
        print("")
        print("Stopped by user.")

    finally:
        print("")
        print("Final STOP...")
        stop(master, 1.5)

        if not args.no_disarm:
            disarm(master)
            stop(master, 1.0)

        print("Forward-only pulse test finished.")


if __name__ == "__main__":
    main()