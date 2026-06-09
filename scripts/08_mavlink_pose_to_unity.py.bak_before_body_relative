import argparse
import math
import socket
import struct
import time

from pymavlink import mavutil


PACKET = struct.Struct("<9f")


def wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def deg(rad):
    return math.degrees(rad)


def maybe_invert(value, invert):
    return -value if invert else value


def main():
    parser = argparse.ArgumentParser(
        description="Forward MAVLink BlueROV2 pose to Unity using the existing 9-float UDP pose format."
    )

    parser.add_argument("--mavlink", default="udpin:127.0.0.1:14552")
    parser.add_argument("--unity-ip", default="192.168.137.1")
    parser.add_argument("--unity-port", type=int, default=5007)

    parser.add_argument("--rate", type=float, default=60.0)
    parser.add_argument("--scale", type=float, default=1.0)

    # Incoming frame sent to Unity receiver:
    # x = forward, y = right, z = up
    # Default mapping from LOCAL_POSITION_NED:
    # forward = NED x
    # right   = NED y
    # up      = -NED z
    parser.add_argument("--invert-x", action="store_true")
    parser.add_argument("--invert-y", action="store_true")
    parser.add_argument("--invert-z", action="store_true")
    parser.add_argument("--swap-xy", action="store_true")

    parser.add_argument("--yaw-sign", type=float, default=1.0)
    parser.add_argument("--roll-sign", type=float, default=1.0)
    parser.add_argument("--pitch-sign", type=float, default=1.0)

    parser.add_argument("--position-only", action="store_true")
    parser.add_argument("--yaw-only", action="store_true")

    parser.add_argument("--print-rate", type=float, default=2.0)
    parser.add_argument("--runtime", type=float, default=None)

    args = parser.parse_args()

    period = 1.0 / args.rate

    print("=== MAVLink Pose to Unity Bridge ===")
    print(f"MAVLink input      : {args.mavlink}")
    print(f"Unity target       : {args.unity_ip}:{args.unity_port}")
    print(f"rate               : {args.rate} Hz")
    print(f"scale              : {args.scale}")
    print(f"yaw_sign           : {args.yaw_sign}")
    print(f"position_only      : {args.position_only}")
    print(f"yaw_only           : {args.yaw_only}")
    print("")

    master = mavutil.mavlink_connection(args.mavlink)

    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print(
        f"Heartbeat received. "
        f"target_system={master.target_system}, target_component={master.target_component}"
    )
    
    def request_message_interval(message_id, rate_hz):
        if rate_hz <= 0:
            return

        interval_us = int(1_000_000 / rate_hz)

        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )

        print(f"Requested MAVLink message {message_id} at {rate_hz} Hz")


    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 30)
    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 30)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    destination = (args.unity_ip, args.unity_port)

    latest_pos = None
    latest_att = None

    origin_pos = None
    origin_roll = None
    origin_pitch = None
    origin_yaw = None

    seq = 0
    start = time.time()
    last_send = time.perf_counter()
    last_print = 0.0

    print("Streaming pose to Unity...")
    print("Press Ctrl+C to stop.")
    print("")

    try:
        while True:
            now = time.time()
            packet_time = now - start

            if args.runtime is not None and (now - start) >= args.runtime:
                break

            # Drain available MAVLink messages and keep only latest pose/attitude.
            while True:
                msg = master.recv_match(
                    type=["LOCAL_POSITION_NED", "ATTITUDE"],
                    blocking=False,
                )

                if msg is None:
                    break

                msg_type = msg.get_type()

                if msg_type == "LOCAL_POSITION_NED":
                    latest_pos = msg

                elif msg_type == "ATTITUDE":
                    latest_att = msg

            if latest_pos is not None and latest_att is not None:
                if origin_pos is None:
                    origin_pos = (
                        float(latest_pos.x),
                        float(latest_pos.y),
                        float(latest_pos.z),
                    )
                    origin_roll = float(latest_att.roll)
                    origin_pitch = float(latest_att.pitch)
                    origin_yaw = float(latest_att.yaw)

                    print(
                        "Origin locked: "
                        f"NED=({origin_pos[0]:.3f}, {origin_pos[1]:.3f}, {origin_pos[2]:.3f}), "
                        f"RPYdeg=({deg(origin_roll):.2f}, {deg(origin_pitch):.2f}, {deg(origin_yaw):.2f})"
                    )

                dx_n = float(latest_pos.x) - origin_pos[0]
                dy_e = float(latest_pos.y) - origin_pos[1]
                dz_d = float(latest_pos.z) - origin_pos[2]

                # Convert NED relative position to existing Python/Unity UDP frame:
                # +X forward, +Y right, +Z up
                x_forward = dx_n
                y_right = dy_e
                z_up = -dz_d

                if args.swap_xy:
                    x_forward, y_right = y_right, x_forward

                x_forward = maybe_invert(x_forward, args.invert_x)
                y_right = maybe_invert(y_right, args.invert_y)
                z_up = maybe_invert(z_up, args.invert_z)

                x_forward *= args.scale
                y_right *= args.scale
                z_up *= args.scale

                roll = wrap_deg(deg(float(latest_att.roll) - origin_roll)) * args.roll_sign
                pitch = wrap_deg(deg(float(latest_att.pitch) - origin_pitch)) * args.pitch_sign
                yaw = wrap_deg(deg(float(latest_att.yaw) - origin_yaw)) * args.yaw_sign

                if args.position_only:
                    roll = 0.0
                    pitch = 0.0
                    yaw = 0.0

                if args.yaw_only:
                    roll = 0.0
                    pitch = 0.0

                perf_now = time.perf_counter()
                sender_dt = perf_now - last_send
                last_send = perf_now

                payload = PACKET.pack(
                    float(x_forward),
                    float(y_right),
                    float(z_up),
                    float(roll),
                    float(pitch),
                    float(yaw),
                    float(packet_time),
                    float(seq),
                    float(sender_dt),
                )

                sock.sendto(payload, destination)

                if now - last_print >= (1.0 / args.print_rate):
                    last_print = now
                    print(
                        f"seq={seq:06d} "
                        f"pos=({x_forward:+.3f}, {y_right:+.3f}, {z_up:+.3f}) "
                        f"rpy=({roll:+.1f}, {pitch:+.1f}, {yaw:+.1f}) "
                        f"raw_ned=({latest_pos.x:+.3f}, {latest_pos.y:+.3f}, {latest_pos.z:+.3f})"
                    )

                seq += 1

            elapsed = time.perf_counter() - last_send
            sleep_time = period - elapsed

            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                time.sleep(0.001)

    except KeyboardInterrupt:
        print("")
        print("Stopped by user.")

    finally:
        sock.close()
        print("")
        print("Pose bridge stopped.")


if __name__ == "__main__":
    main()
