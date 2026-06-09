import argparse
import socket
import struct
import time


def send_packet(sock, ip, port, x, y, z, roll, pitch, yaw, seq, sender_dt):
    now = time.time()
    payload = struct.pack(
        "<9f",
        float(x),
        float(y),
        float(z),
        float(roll),
        float(pitch),
        float(yaw),
        float(now),
        float(seq),
        float(sender_dt),
    )
    sock.sendto(payload, (ip, port))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--unity-ip", required=True)
    parser.add_argument("--unity-port", type=int, default=5008)
    parser.add_argument("--rate", type=float, default=30.0)

    parser.add_argument("--axis", choices=["x", "y", "z"], required=True)
    parser.add_argument("--amount", type=float, default=1.0)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--pause", type=float, default=2.0)

    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=0.0)

    args = parser.parse_args()

    dt = 1.0 / args.rate
    seq = 0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_for(seconds, x, y, z):
        nonlocal seq
        n = int(seconds * args.rate)
        for _ in range(n):
            send_packet(
                sock,
                args.unity_ip,
                args.unity_port,
                x,
                y,
                z,
                args.roll,
                args.pitch,
                args.yaw,
                seq,
                dt,
            )
            seq += 1
            time.sleep(dt)

    print("=== Unity Pose Axis Probe ===")
    print(f"Target       : {args.unity_ip}:{args.unity_port}")
    print(f"Axis         : {args.axis}")
    print(f"Amount       : {args.amount}")
    print(f"Duration     : {args.duration}")
    print(f"Pause        : {args.pause}")
    print(f"Yaw/Roll/Pitch: {args.yaw}, {args.roll}, {args.pitch}")
    print("")

    print("Phase 1: origin / neutral")
    send_for(args.pause, 0.0, 0.0, 0.0)

    print("Phase 2: positive axis ramp/hold")
    if args.axis == "x":
        send_for(args.duration, args.amount, 0.0, 0.0)
    elif args.axis == "y":
        send_for(args.duration, 0.0, args.amount, 0.0)
    else:
        send_for(args.duration, 0.0, 0.0, args.amount)

    print("Phase 3: back to origin")
    send_for(args.pause, 0.0, 0.0, 0.0)

    print("Phase 4: negative axis ramp/hold")
    if args.axis == "x":
        send_for(args.duration, -args.amount, 0.0, 0.0)
    elif args.axis == "y":
        send_for(args.duration, 0.0, -args.amount, 0.0)
    else:
        send_for(args.duration, 0.0, 0.0, -args.amount)

    print("Phase 5: back to origin")
    send_for(args.pause, 0.0, 0.0, 0.0)

    sock.close()
    print("Axis probe finished.")


if __name__ == "__main__":
    main()