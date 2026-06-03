import argparse
import math
import time
from pymavlink import mavutil


DEFAULT_MAVLINK = "udpin:127.0.0.1:14551"
RATE_HZ = 20
Z_NEUTRAL = 500


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def send_manual(master, x, y, z, r):
    master.mav.manual_control_send(
        master.target_system,
        int(x),
        int(y),
        int(z),
        int(r),
        0,
    )


def send_for(master, x, y, z, r, duration, rate_hz=RATE_HZ):
    n = int(duration * rate_hz)
    dt = 1.0 / rate_hz

    for _ in range(n):
        send_manual(master, x, y, z, r)
        time.sleep(dt)


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
        print(
            f"HEARTBEAT armed={armed} "
            f"base_mode={msg.base_mode} custom_mode={msg.custom_mode}"
        )

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

    if not wait_armed(master, expected=True):
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

    wait_armed(master, expected=False)


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


def read_status(master, duration=0.8):
    end = time.time() + duration

    while time.time() < end:
        msg = master.recv_match(
            type=["ATTITUDE", "LOCAL_POSITION_NED", "HEARTBEAT"],
            blocking=True,
            timeout=0.2,
        )

        if msg is None:
            continue

        t = msg.get_type()

        if t == "ATTITUDE":
            yaw_deg = math.degrees(msg.yaw)
            print(
                f"  ATT roll={math.degrees(msg.roll):+.2f}deg "
                f"pitch={math.degrees(msg.pitch):+.2f}deg "
                f"yaw={yaw_deg:+.2f}deg"
            )

        elif t == "LOCAL_POSITION_NED":
            print(
                f"  POS x={msg.x:+.3f} y={msg.y:+.3f} z={msg.z:+.3f} "
                f"vx={msg.vx:+.3f} vy={msg.vy:+.3f} vz={msg.vz:+.3f}"
            )

        elif t == "HEARTBEAT":
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            print(f"  HB armed={armed} mode={msg.custom_mode}")


def yaw_pulse(master, name, r, duration, pause):
    r = int(clamp(r, -1000, 1000))

    print("")
    print("=" * 70)
    print(f"{name}: x=0, y=0, z={Z_NEUTRAL}, r={r}, duration={duration:.1f}s")
    print("=" * 70)

    print("Before pulse:")
    read_status(master, 0.6)

    print(f"Sending yaw command r={r}...")
    send_for(master, 0, 0, Z_NEUTRAL, r, duration)

    print("After pulse:")
    read_status(master, 0.8)

    stop(master, pause)

    print("After stop:")
    read_status(master, 0.6)


def main():
    parser = argparse.ArgumentParser(
        description="Yaw-only MAVLink MANUAL_CONTROL pulse test for BlueROV2 / ArduSub."
    )

    parser.add_argument("--mavlink", default=DEFAULT_MAVLINK)
    parser.add_argument("--r", type=int, default=80)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--pause", type=float, default=2.0)
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--start-negative", action="store_true")
    parser.add_argument("--no-disarm", action="store_true")

    args = parser.parse_args()

    r = int(clamp(args.r, 0, 1000))

    if r == 0:
        raise ValueError("--r must be nonzero. Example: --r 80")

    print("=== Yaw Only Pulse Test ===")
    print(f"MAVLink   : {args.mavlink}")
    print(f"r command : +/-{r}")
    print(f"duration  : {args.duration:.1f}s")
    print(f"pause     : {args.pause:.1f}s")
    print(f"cycles    : {args.cycles}")
    print(f"z neutral : {Z_NEUTRAL}")
    print("Forward/side/vertical commands are disabled: x=0, y=0, z=500")
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
                yaw_pulse(master, "YAW_LEFT_OR_NEGATIVE_R", -r, args.duration, args.pause)
                yaw_pulse(master, "YAW_RIGHT_OR_POSITIVE_R", +r, args.duration, args.pause)
            else:
                yaw_pulse(master, "YAW_RIGHT_OR_POSITIVE_R", +r, args.duration, args.pause)
                yaw_pulse(master, "YAW_LEFT_OR_NEGATIVE_R", -r, args.duration, args.pause)

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

        print("Yaw-only pulse test finished.")


if __name__ == "__main__":
    main()