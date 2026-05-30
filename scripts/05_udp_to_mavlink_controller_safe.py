import argparse
import json
import socket
import time

from pymavlink import mavutil


ARMED_FLAG = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED


def clamp(value, low, high):
    return max(low, min(high, value))


def is_heartbeat_armed(msg):
    if msg is None or msg.get_type() != "HEARTBEAT":
        return False

    base_mode = msg.to_dict().get("base_mode", 0)
    return (base_mode & ARMED_FLAG) != 0


def wait_arm_state(master, desired_armed, timeout=10):
    start = time.time()

    while time.time() - start < timeout:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)

        if msg is None:
            continue

        armed = is_heartbeat_armed(msg)
        data = msg.to_dict()

        print(
            f"HEARTBEAT: armed={armed} "
            f"base_mode={data.get('base_mode')} "
            f"custom_mode={data.get('custom_mode')} "
            f"system_status={data.get('system_status')}"
        )

        if armed == desired_armed:
            return True

    return False


def set_mode_manual(master, target_system):
    mode_mapping = master.mode_mapping()

    print("Available modes:", list(mode_mapping.keys()))

    if "MANUAL" not in mode_mapping:
        raise RuntimeError("MANUAL mode not found.")

    mode_id = mode_mapping["MANUAL"]

    print(f"Requesting MANUAL mode. mode_id={mode_id}")

    master.mav.set_mode_send(
        target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    time.sleep(2)


def arm_vehicle(master, target_system, target_component):
    print("Sending ARM command...")

    master.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0
    )

    print("Waiting for armed state...")
    ok = wait_arm_state(master, desired_armed=True, timeout=10)

    if ok:
        print("Vehicle is ARMED.")
    else:
        print("WARNING: Vehicle did not report ARMED state.")

    return ok


def disarm_vehicle(master, target_system, target_component):
    print("Sending DISARM command...")

    master.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
    )

    print("Waiting for disarmed state...")
    ok = wait_arm_state(master, desired_armed=False, timeout=10)

    if ok:
        print("Vehicle is DISARMED.")
    else:
        print("WARNING: Vehicle did not report DISARMED state.")

    return ok


def send_manual_control(master, target_system, x=0, y=0, z=500, r=0):
    master.mav.manual_control_send(
        target_system,
        int(x),
        int(y),
        int(z),
        int(r),
        0
    )


def send_stop_for(master, target_system, duration=1.0, hz=20):
    period = 1.0 / hz
    end_time = time.time() + duration
    count = 0

    while time.time() < end_time:
        send_manual_control(master, target_system, x=0, y=0, z=500, r=0)
        count += 1
        time.sleep(period)

    print(f"STOP messages sent: {count}")


def packet_to_command(pkt, args):
    valid = pkt.get("valid", False)
    face_id = pkt.get("face_id", None)

    pattern_accuracy = float(pkt.get("pattern_accuracy", 0.0))
    distance_confidence = float(pkt.get("distance_confidence", 0.0))

    if not valid:
        return 0, 0, 500, 0, "INVALID"

    if face_id != "BACK":
        return 0, 0, 500, 0, "WRONG_FACE"

    if pattern_accuracy < args.pattern_acc_min:
        return 0, 0, 500, 0, "LOW_PATTERN_CONF"

    error_norm = pkt.get("error_norm", [0.0, 0.0])
    estimated_distance = pkt.get("estimated_distance", None)

    error_x = float(error_norm[0])

    # 03_manual_control_axis_test sonucuna göre doğrulandı:
    # r > 0 -> sağa yaw
    # r < 0 -> sola yaw
    # error_x > 0 -> hedef görüntü merkezinin sağında
    r = clamp(args.k_yaw * error_x, -args.max_r, args.max_r)

    # İlk güvenli testte vertical kapalı.
    y = 0
    z = 500

    if estimated_distance is None:
        x = 0
        return int(x), int(y), int(z), int(r), "ALIGN_ONLY_NO_DISTANCE"

    if distance_confidence < args.dist_conf_min:
        x = 0
        return int(x), int(y), int(z), int(r), "ALIGN_ONLY_LOW_DISTANCE_CONF"

    distance_error = float(estimated_distance) - args.desired_distance

    # 03_manual_control_axis_test sonucuna göre doğrulandı:
    # x > 0 -> robot baktığı yöne ileri
    # x < 0 -> robot geri
    # distance_error > 0 -> hedef uzakta
    x = clamp(args.k_forward * distance_error, -args.max_x, args.max_x)

    return int(x), int(y), int(z), int(r), "TRACK"


def parse_udp_packet(data):
    return json.loads(data.decode("utf-8"))


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--mavlink", default="udpin:127.0.0.1:14551")
    parser.add_argument("--udp-host", default="0.0.0.0")
    parser.add_argument("--udp-port", type=int, default=5005)

    parser.add_argument("--runtime", type=float, default=15.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--packet-timeout", type=float, default=0.5)

    parser.add_argument("--desired-distance", type=float, default=3.0)

    parser.add_argument("--k-forward", type=float, default=400.0)
    parser.add_argument("--k-yaw", type=float, default=500.0)

    parser.add_argument("--max-x", type=float, default=300.0)
    parser.add_argument("--max-r", type=float, default=300.0)

    parser.add_argument("--pattern-acc-min", type=float, default=0.95)
    parser.add_argument("--dist-conf-min", type=float, default=0.60)

    parser.add_argument(
        "--arm",
        action="store_true",
        help="If set, the vehicle will be armed and MANUAL_CONTROL will affect motion."
    )

    args = parser.parse_args()

    print("=== UDP to MAVLink SAFE Controller ===")
    print(f"MAVLink connection : {args.mavlink}")
    print(f"UDP listen         : {args.udp_host}:{args.udp_port}")
    print(f"runtime            : {args.runtime}s")
    print(f"rate               : {args.rate} Hz")
    print(f"packet_timeout     : {args.packet_timeout}s")
    print(f"desired_distance   : {args.desired_distance}")
    print(f"k_forward          : {args.k_forward}")
    print(f"k_yaw              : {args.k_yaw}")
    print(f"max_x              : {args.max_x}")
    print(f"max_r              : {args.max_r}")
    print(f"arm enabled        : {args.arm}")
    print("Vertical control   : disabled, z=500 fixed")
    print("")

    print(f"Connecting to MAVLink: {args.mavlink}")
    master = mavutil.mavlink_connection(args.mavlink)

    print("Waiting for heartbeat...")
    heartbeat = master.wait_heartbeat(timeout=30)

    if heartbeat is None:
        print("ERROR: No MAVLink heartbeat received.")
        return

    target_system = master.target_system
    target_component = master.target_component

    print("Heartbeat received.")
    print(f"target_system    = {target_system}")
    print(f"target_component = {target_component}")

    print("")
    print("Opening UDP socket...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.udp_host, args.udp_port))
    sock.setblocking(False)

    print(f"Listening UDP on {args.udp_host}:{args.udp_port}")
    print("")

    armed_by_script = False

    last_packet = None
    last_packet_time = None
    last_seq = None

    current_x = 0
    current_y = 0
    current_z = 500
    current_r = 0
    current_state = "NO_PACKET"

    period = 1.0 / args.rate

    try:
        set_mode_manual(master, target_system)

        print("Sending neutral MANUAL_CONTROL before arm...")
        send_stop_for(master, target_system, duration=1.5, hz=int(args.rate))

        if args.arm:
            armed_ok = arm_vehicle(master, target_system, target_component)

            if not armed_ok:
                print("ERROR: Vehicle could not be armed. Exiting safely.")
                return

            armed_by_script = True

            print("Sending neutral MANUAL_CONTROL after arm...")
            send_stop_for(master, target_system, duration=1.5, hz=int(args.rate))
        else:
            print("ARM is disabled. This run will not move the vehicle.")
            print("Use --arm only after dry safety check is OK.")

        print("")
        print("Starting control loop...")
        print("Press Ctrl+C to stop early.")
        print("")

        start_time = time.time()
        next_print = time.time()

        while time.time() - start_time < args.runtime:
            loop_start = time.time()

            # 1) UDP paketlerini bloklamadan oku.
            while True:
                try:
                    data, addr = sock.recvfrom(8192)
                except BlockingIOError:
                    break

                try:
                    pkt = parse_udp_packet(data)
                except Exception as e:
                    print(f"UDP JSON parse error: {e}")
                    continue

                now = time.time()
                seq = pkt.get("udp_seq", None)

                if last_seq is not None and seq is not None:
                    if seq != last_seq + 1:
                        # Sender yeniden başlarsa 49 -> 0 normaldir.
                        if not (last_seq >= 0 and seq == 0):
                            print(f"WARNING: sequence jump. last_seq={last_seq}, current_seq={seq}")

                if seq is not None:
                    last_seq = seq

                last_packet = pkt
                last_packet_time = now

            # 2) Paket taze mi kontrol et.
            now = time.time()

            if last_packet is None:
                current_x, current_y, current_z, current_r = 0, 0, 500, 0
                current_state = "NO_PACKET"

            elif last_packet_time is not None and (now - last_packet_time) > args.packet_timeout:
                current_x, current_y, current_z, current_r = 0, 0, 500, 0
                current_state = "PACKET_TIMEOUT"

            else:
                current_x, current_y, current_z, current_r, current_state = packet_to_command(
                    last_packet,
                    args
                )

            # 3) Kritik nokta:
            # Araç armed ise veya değilse bile MANUAL_CONTROL sürekli gönderilir.
            send_manual_control(
                master,
                target_system,
                x=current_x,
                y=current_y,
                z=current_z,
                r=current_r
            )

            # 4) MAVLink status mesajlarını bloklamadan oku.
            msg = master.recv_match(
                type=["HEARTBEAT", "VFR_HUD", "LOCAL_POSITION_NED"],
                blocking=False
            )

            # 5) Debug print.
            if now >= next_print:
                seq = None if last_packet is None else last_packet.get("udp_seq", None)
                valid = None if last_packet is None else last_packet.get("valid", None)
                face = None if last_packet is None else last_packet.get("face_id", None)
                err = None if last_packet is None else last_packet.get("error_norm", None)
                dist = None if last_packet is None else last_packet.get("estimated_distance", None)

                age = None
                if last_packet_time is not None:
                    age = now - last_packet_time

                mav_text = ""

                if msg is not None:
                    data = msg.to_dict()
                    if msg.get_type() == "HEARTBEAT":
                        armed = (data.get("base_mode", 0) & ARMED_FLAG) != 0
                        mav_text = f" | HB armed={armed} mode={data.get('custom_mode')}"
                    elif msg.get_type() == "VFR_HUD":
                        mav_text = (
                            f" | HUD gs={data.get('groundspeed'):.3f} "
                            f"thr={data.get('throttle')} alt={data.get('alt'):.3f}"
                        )
                    elif msg.get_type() == "LOCAL_POSITION_NED":
                        mav_text = (
                            f" | POS x={data.get('x'):.2f} "
                            f"y={data.get('y'):.2f} z={data.get('z'):.2f}"
                        )

                age_text = "None" if age is None else f"{age:.3f}"

                print(
                    f"state={current_state} "
                    f"seq={seq} "
                    f"valid={valid} "
                    f"face={face} "
                    f"age={age_text}s "
                    f"err={err} "
                    f"dist={dist} "
                    f"cmd=({current_x},{current_y},{current_z},{current_r})"
                    f"{mav_text}"
                )

                next_print = now + 0.5

            # 6) Loop rate sabitle.
            elapsed = time.time() - loop_start
            sleep_time = period - elapsed

            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("")
        print("KeyboardInterrupt received. Stopping safely...")

    finally:
        print("")
        print("Sending STOP before exit...")
        send_stop_for(master, target_system, duration=1.5, hz=int(args.rate))

        if armed_by_script:
            disarm_vehicle(master, target_system, target_component)

        print("Controller finished safely.")


if __name__ == "__main__":
    main()
