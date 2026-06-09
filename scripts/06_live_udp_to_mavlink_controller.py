import argparse
import csv
import json
import math
import socket
import time
from datetime import datetime
from pathlib import Path

from pymavlink import mavutil


MAV_MODE_FLAG_SAFETY_ARMED = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def sign_preserving_deadband(value, deadband):
    if abs(value) < deadband:
        return 0.0
    return value


def is_armed_from_heartbeat(msg):
    if msg is None:
        return False
    return (msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0


def wait_armed_state(master, expected_armed, timeout_s=10.0):
    start = time.time()

    while time.time() - start < timeout_s:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)

        if msg is None:
            continue

        armed = is_armed_from_heartbeat(msg)

        print(
            f"HEARTBEAT: armed={armed} "
            f"base_mode={msg.base_mode} "
            f"custom_mode={msg.custom_mode} "
            f"system_status={msg.system_status}"
        )

        if armed == expected_armed:
            return True

    return False


def connect_mavlink(connection_string):
    print(f"Connecting to MAVLink: {connection_string}")
    master = mavutil.mavlink_connection(connection_string)

    print("Waiting for heartbeat...")
    master.wait_heartbeat()

    print("Heartbeat received.")
    print(f"target_system    = {master.target_system}")
    print(f"target_component = {master.target_component}")

    return master


def request_mode(master, mode_name):
    mode_mapping = master.mode_mapping()

    if mode_mapping is None:
        raise RuntimeError("Could not get mode mapping from MAVLink vehicle.")

    print(f"Available modes: {list(mode_mapping.keys())}")

    if mode_name not in mode_mapping:
        raise RuntimeError(f"Mode {mode_name!r} not available. Available: {list(mode_mapping.keys())}")

    mode_id = mode_mapping[mode_name]

    print(f"Requesting {mode_name} mode. mode_id={mode_id}")

    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )

    time.sleep(1.0)


def send_manual_control(master, x, y, z, r, buttons=0):
    master.mav.manual_control_send(
        master.target_system,
        int(x),
        int(y),
        int(z),
        int(r),
        int(buttons),
    )


def send_stop(master, duration_s=1.0, rate_hz=20.0, z_neutral=500):
    count = int(duration_s * rate_hz)

    for _ in range(count):
        send_manual_control(master, 0, 0, z_neutral, 0)
        time.sleep(1.0 / rate_hz)

    print(f"STOP messages sent: {count}")


def arm_vehicle(master):
    print("Sending ARM command...")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )

    print("Waiting for armed state...")

    if not wait_armed_state(master, expected_armed=True, timeout_s=10.0):
        raise RuntimeError("Vehicle did not arm within timeout.")

    print("Vehicle is ARMED.")


def disarm_vehicle(master):
    print("Sending DISARM command...")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )

    print("Waiting for disarmed state...")

    if not wait_armed_state(master, expected_armed=False, timeout_s=10.0):
        print("WARNING: Vehicle did not disarm within timeout.")
        return False

    print("Vehicle is DISARMED.")
    return True


def parse_observation_packet(data):
    try:
        text = data.decode("utf-8")
        packet = json.loads(text)

        if not isinstance(packet, dict):
            return None, "JSON_NOT_OBJECT"

        return packet, None

    except UnicodeDecodeError:
        return None, "DECODE_ERROR"
    except json.JSONDecodeError:
        return None, "JSON_ERROR"


def read_latest_udp_packet(sock):
    latest_packet = None
    latest_error = None
    latest_addr = None

    while True:
        try:
            data, addr = sock.recvfrom(65535)
        except BlockingIOError:
            break

        packet, error = parse_observation_packet(data)

        latest_packet = packet
        latest_error = error
        latest_addr = addr

    return latest_packet, latest_error, latest_addr


def validate_observation(packet, args):
    if packet is None:
        return False, "NO_PACKET"

    if not packet.get("valid", False):
        return False, "PACKET_INVALID"

    face_id = packet.get("face_id")

    if face_id != args.required_face:
        return False, "WRONG_FACE"

    pattern_accuracy = packet.get("pattern_accuracy")
    distance_confidence = packet.get("distance_confidence")
    error_norm = packet.get("error_norm")
    estimated_distance = packet.get("estimated_distance")

    try:
        pattern_accuracy = float(pattern_accuracy)
        distance_confidence = float(distance_confidence)
        estimated_distance = float(estimated_distance)
    except (TypeError, ValueError):
        return False, "MISSING_NUMERIC_FIELD"

    if pattern_accuracy < args.min_pattern_accuracy:
        return False, "LOW_PATTERN_ACCURACY"

    if distance_confidence < args.min_distance_confidence:
        return False, "LOW_DISTANCE_CONFIDENCE"

    if not isinstance(error_norm, list) or len(error_norm) < 2:
        return False, "BAD_ERROR_NORM"

    try:
        error_x = float(error_norm[0])
        error_y = float(error_norm[1])
    except (TypeError, ValueError):
        return False, "BAD_ERROR_NORM"

    if not math.isfinite(error_x) or not math.isfinite(error_y):
        return False, "NONFINITE_ERROR"

    if not math.isfinite(estimated_distance):
        return False, "NONFINITE_DISTANCE"

    return True, "OK"

_yaw_d_state = {
    "prev_error_x_smooth": None,
    "error_x_smooth": None,
}

def compute_target_command(packet, args, dt):
    error_x = float(packet["error_norm"][0])
    estimated_distance = float(packet["estimated_distance"])

    distance_error = estimated_distance - args.desired_distance

    # ------------------------------------------------------------
    # Yaw D term:
    # Smooth error_x first, then differentiate the smoothed signal.
    # This avoids amplifying single-frame OpenCV noise.
    # ------------------------------------------------------------
    alpha_error = clamp(args.error_ema_alpha, 0.0, 1.0)

    if _yaw_d_state["error_x_smooth"] is None:
        _yaw_d_state["error_x_smooth"] = error_x

    error_x_smooth = (
        alpha_error * error_x
        + (1.0 - alpha_error) * _yaw_d_state["error_x_smooth"]
    )

    prev_error_x_smooth = _yaw_d_state["prev_error_x_smooth"]

    if prev_error_x_smooth is None or dt <= 0.0:
        error_x_rate = 0.0
    else:
        error_x_rate = (error_x_smooth - prev_error_x_smooth) / dt

    _yaw_d_state["prev_error_x_smooth"] = error_x_smooth
    _yaw_d_state["error_x_smooth"] = error_x_smooth

    error_x_db = sign_preserving_deadband(error_x, args.yaw_deadband)
    distance_error_db = sign_preserving_deadband(distance_error, args.forward_deadband)

    # P + D yaw control.
    #
    # IMPORTANT SIGN:
    # If error_x > 0 and the robot turns correctly, error_x decreases,
    # so error_x_rate becomes negative.
    # With "+ k_yaw_d * error_x_rate", the D term reduces yaw command
    # while approaching the center, damping overshoot.
    # Distance-dependent yaw gain:
    # Far target  -> weaker yaw to reduce oscillation/noisy correction.
    # Near target -> full yaw gain for precise centering.
    if args.yaw_gain_distance_enable:
        near_d = args.yaw_gain_near_distance
        far_d = args.yaw_gain_far_distance

        if far_d <= near_d:
            yaw_gain_scale = 1.0
        else:
            t = (far_d - estimated_distance) / (far_d - near_d)
            t = clamp(t, 0.0, 1.0)

            yaw_gain_scale = (
                args.yaw_gain_far_scale
                + t * (args.yaw_gain_near_scale - args.yaw_gain_far_scale)
            )
    else:
        yaw_gain_scale = 1.0

    effective_k_yaw = args.k_yaw * yaw_gain_scale
    effective_k_yaw_d = args.k_yaw_d * yaw_gain_scale

    target_r = args.yaw_sign * (
        effective_k_yaw * error_x_db
        + effective_k_yaw_d * error_x_rate
    )

    # Yaw reverse guard:
    # When the target is clearly off-center, the D term may reduce/brake yaw,
    # but it must not actively turn the robot away from the target.
    # Uses raw error_x sign, not deadbanded error_x.
    if args.yaw_reverse_guard_error > 0.0 and abs(error_x) > args.yaw_reverse_guard_error:
        desired_dir = args.yaw_sign * error_x

        if desired_dir > 0.0 and target_r < 0.0:
            target_r = 0.0
        elif desired_dir < 0.0 and target_r > 0.0:
            target_r = 0.0

    target_x = args.k_forward * distance_error_db

    target_x = clamp(target_x, -args.max_x, args.max_x)
    target_r = clamp(target_r, -args.max_r, args.max_r)

    # Yaw-priority / forward-gating:
    # If the visual target is far from image center, reduce or stop forward motion.
    abs_error_x = abs(error_x)

    gate_mid_scale = args.forward_gate_mid_scale
    gate_min_scale = args.forward_gate_min_scale

    if args.forward_gate_distance_enable:
        near_d = args.forward_gate_near_distance
        far_d = args.forward_gate_far_distance

        if far_d > near_d:
            t = (estimated_distance - near_d) / (far_d - near_d)
            t = clamp(t, 0.0, 1.0)

            gate_mid_scale = (
                args.forward_gate_mid_scale
                + t * (args.forward_gate_far_mid_scale - args.forward_gate_mid_scale)
            )

            gate_min_scale = (
                args.forward_gate_min_scale
                + t * (args.forward_gate_far_min_scale - args.forward_gate_min_scale)
            )


    if abs(error_x) >= args.forward_gate_stop_error:
        forward_gate_scale = gate_min_scale
    elif abs(error_x) >= args.forward_gate_slow_error:
        forward_gate_scale = gate_mid_scale
    elif abs(error_x) >= args.forward_gate_start_error:
        forward_gate_scale = gate_mid_scale
    else:
        forward_gate_scale = 1.0

    # Apply yaw-priority forward gate.
    # Only reduce positive forward motion; do not weaken backward correction if too close.
    if target_x > 0.0:
        target_x *= forward_gate_scale


    # Distance-dependent forward boost:
    # Far target  -> allow stronger forward motion.
    # Near target -> no boost, keep precise/slow approach.
    if args.forward_boost_distance_enable and distance_error > 0.0 and target_x > 0.0:
        near_d = args.forward_boost_near_distance
        far_d = args.forward_boost_far_distance

        if far_d <= near_d:
            forward_boost_scale = 1.0
        else:
            t = (estimated_distance - near_d) / (far_d - near_d)
            t = clamp(t, 0.0, 1.0)

            forward_boost_scale = (
                args.forward_boost_near_scale
                + t * (args.forward_boost_far_scale - args.forward_boost_near_scale)
            )

        target_x = clamp(
            target_x * forward_boost_scale,
            0.0,
            args.max_x,
        )
    
    # Held observations are stale visual measurements.
    # Apply held scaling LAST so stale packets cannot be boosted again.
    held = packet_bool(packet.get("held_observation", packet.get("held", False)))
    if held:
        target_x *= args.held_forward_scale
        target_r *= args.held_yaw_scale

    return target_x, target_r, error_x, distance_error


def limit_rate(previous, desired, max_delta):
    delta = desired - previous
    delta = clamp(delta, -max_delta, max_delta)
    return previous + delta


def round_manual_command(value):
    if value > 0.0:
        return int(math.floor(value + 0.5))
    if value < 0.0:
        return int(math.ceil(value - 0.5))
    return 0


def packet_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "y")
    return bool(value)


class CommandFilter:
    def __init__(self, args):
        self.args = args
        self.x = 0.0
        self.r = 0.0

    def reset(self):
        self.x = 0.0
        self.r = 0.0

    def update(self, target_x, target_r, dt, hard_stop=False):
        if hard_stop:
            self.reset()
            return self.x, self.r

        alpha = clamp(self.args.ema_alpha, 0.0, 1.0)

        ema_x = alpha * target_x + (1.0 - alpha) * self.x
        ema_r = alpha * target_r + (1.0 - alpha) * self.r

        max_dx = self.args.max_delta_x_per_sec * dt
        max_dr = self.args.max_delta_r_per_sec * dt

        self.x = limit_rate(self.x, ema_x, max_dx)
        self.r = limit_rate(self.r, ema_r, max_dr)

        # Only zero the internal filter state when the corresponding target is zero.
        # Otherwise small but intentional commands can never accumulate.
        if abs(target_x) < 1e-9 and abs(self.x) < 0.5:
            self.x = 0.0

        if abs(target_r) < 1e-9 and abs(self.r) < 0.5:
            self.r = 0.0

        return self.x, self.r


def open_csv_log(path):
    if path is None:
        return None, None

    log_path = Path(path)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    f = log_path.open("w", encoding="utf-8", newline="")

    fieldnames = [
        "time_unix",
        "state",
        "udp_seq",
        "valid",
        "reason",
        "packet_age_s",
        "invalid_duration_s",
        "error_x",
        "distance_error",
        "target_x",
        "target_r",
        "smooth_x",
        "smooth_r",
        "sent_x",
        "sent_y",
        "sent_z",
        "sent_r",
        "estimated_distance",
        "distance_confidence",
        "held_observation",
        "packet_reason",
    ]

    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()

    return f, writer


def write_csv_row(writer, row):
    if writer is not None:
        writer.writerow(row)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--mavlink", default="udpin:127.0.0.1:14551")
    parser.add_argument("--udp-host", default="0.0.0.0")
    parser.add_argument("--udp-port", type=int, default=5005)

    parser.add_argument("--runtime", type=float, default=30.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--packet-timeout", type=float, default=1.0)

    parser.add_argument("--mode", default="MANUAL")
    parser.add_argument("--arm", action="store_true")

    parser.add_argument("--required-face", default="BACK")

    parser.add_argument("--desired-distance", type=float, default=3.0)

    parser.add_argument("--k-forward", type=float, default=100.0)
    parser.add_argument("--k-yaw", type=float, default=120.0)
    parser.add_argument(
        "--yaw-sign",
        type=float,
        default=1.0,
        help="Yaw command sign. Use 1.0 for normal mapping, -1.0 to invert yaw direction."
    )

    parser.add_argument("--max-x", type=float, default=120.0)
    parser.add_argument("--max-r", type=float, default=120.0)

    parser.add_argument("--z-neutral", type=int, default=500)

    parser.add_argument("--min-pattern-accuracy", type=float, default=0.90)
    parser.add_argument("--min-distance-confidence", type=float, default=0.60)

    parser.add_argument("--yaw-deadband", type=float, default=0.04)
    parser.add_argument("--forward-deadband", type=float, default=0.15)

    parser.add_argument("--forward-gate-start-error", type=float, default=0.12)
    parser.add_argument("--forward-gate-slow-error", type=float, default=0.20)
    parser.add_argument("--forward-gate-stop-error", type=float, default=0.30)
    parser.add_argument("--forward-gate-mid-scale", type=float, default=0.60)
    parser.add_argument("--forward-gate-min-scale", type=float, default=0.30)

    parser.add_argument("--held-forward-scale", type=float, default=0.0)
    parser.add_argument("--held-yaw-scale", type=float, default=0.4)

    parser.add_argument("--ema-alpha", type=float, default=0.25)
    

    parser.add_argument(
        "--k-yaw-d",
        type=float,
        default=0.0,
        help="Yaw derivative gain. 0 disables D term and keeps old pure-P behavior.",
    )
    parser.add_argument(
        "--yaw-reverse-guard-error",
        type=float,
        default=0.07,
        help="Above this |error_x|, the D term may reduce but not reverse yaw direction. 0 disables guard.",
    )

    parser.add_argument(
        "--error-ema-alpha",
        type=float,
        default=0.4,
        help="EMA alpha for smoothing error_x before derivative calculation.",
    )
    parser.add_argument(
        "--yaw-gain-distance-enable",
        action="store_true",
        help="Enable distance-dependent yaw gain scaling.",
    )

    parser.add_argument(
        "--yaw-gain-near-distance",
        type=float,
        default=2.4,
        help="At or below this distance, yaw gain uses near scale.",
    )

    parser.add_argument(
        "--yaw-gain-far-distance",
        type=float,
        default=4.0,
        help="At or above this distance, yaw gain uses far scale.",
    )

    parser.add_argument(
        "--yaw-gain-near-scale",
        type=float,
        default=1.0,
        help="Yaw gain scale at near distance.",
    )

    parser.add_argument(
        "--yaw-gain-far-scale",
        type=float,
        default=0.60,
        help="Yaw gain scale at far distance.",
    )

    parser.add_argument("--max-delta-x-per-sec", type=float, default=180.0)
    parser.add_argument("--max-delta-r-per-sec", type=float, default=220.0)

    parser.add_argument("--invalid-decay-seconds", type=float, default=0.45)

    parser.add_argument("--print-rate", type=float, default=2.0)

    parser.add_argument("--log-csv", default=None)
    
    parser.add_argument("--seq-jump-warning-threshold", type=int, default=10)
    
    parser.add_argument(
        "--hard-stop-on-held",
        action="store_true",
        help="Immediately reset command filter and stop when observation is held/stale.",
    )

    parser.add_argument(
        "--forward-boost-distance-enable",
        action="store_true",
        help="Enable distance-dependent forward boost.",
    )

    parser.add_argument(
        "--forward-boost-near-distance",
        type=float,
        default=2.8,
        help="At or below this distance, forward boost uses near scale.",
    )

    parser.add_argument(
        "--forward-boost-far-distance",
        type=float,
        default=4.3,
        help="At or above this distance, forward boost uses far scale.",
    )

    parser.add_argument(
        "--forward-boost-near-scale",
        type=float,
        default=1.0,
        help="Forward boost scale at near distance.",
    )

    parser.add_argument(
        "--forward-boost-far-scale",
        type=float,
        default=1.5,
        help="Forward boost scale at far distance.",
    )
    parser.add_argument(
        "--forward-gate-distance-enable",
        action="store_true",
        help="Enable distance-dependent forward gate scales.",
    )

    parser.add_argument(
        "--forward-gate-near-distance",
        type=float,
        default=2.4,
        help="At or below this distance, use normal near forward gate scales.",
    )

    parser.add_argument(
        "--forward-gate-far-distance",
        type=float,
        default=4.3,
        help="At or above this distance, use far forward gate scales.",
    )

    parser.add_argument(
        "--forward-gate-far-mid-scale",
        type=float,
        default=0.90,
        help="Forward gate mid scale at far distance.",
    )

    parser.add_argument(
        "--forward-gate-far-min-scale",
        type=float,
        default=0.70,
        help="Forward gate min scale at far distance.",
    )

    args = parser.parse_args()

    print("=== Live UDP to MAVLink Controller V2 ===")
    print(f"MAVLink connection       : {args.mavlink}")
    print(f"UDP listen               : {args.udp_host}:{args.udp_port}")
    print(f"runtime                  : {args.runtime}s")
    print(f"rate                     : {args.rate} Hz")
    print(f"packet_timeout           : {args.packet_timeout}s")
    print(f"desired_distance         : {args.desired_distance}")
    print(f"k_forward                : {args.k_forward}")
    print(f"k_yaw                    : {args.k_yaw}")
    print(f"k_yaw_d                  : {args.k_yaw_d}")
    print(f"yaw_reverse_guard_error  : {args.yaw_reverse_guard_error}")
    print(f"error_ema_alpha          : {args.error_ema_alpha}")
    print(f"yaw_sign                 : {args.yaw_sign}")
    print(f"max_x                    : {args.max_x}")
    print(f"max_r                    : {args.max_r}")
    print(f"yaw_deadband             : {args.yaw_deadband}")
    print(f"forward_deadband         : {args.forward_deadband}")
    print(f"ema_alpha                : {args.ema_alpha}")
    print(f"max_delta_x_per_sec      : {args.max_delta_x_per_sec}")
    print(f"max_delta_r_per_sec      : {args.max_delta_r_per_sec}")
    print(f"invalid_decay_seconds    : {args.invalid_decay_seconds}")
    print(f"arm enabled              : {args.arm}")
    print(f"Vertical control         : disabled, z={args.z_neutral} fixed")
    print(f"hard_stop_on_held        : {args.hard_stop_on_held}")
    print(f"yaw_gain_distance_enable : {args.yaw_gain_distance_enable}")
    print(f"yaw_gain_near_distance   : {args.yaw_gain_near_distance}")
    print(f"yaw_gain_far_distance    : {args.yaw_gain_far_distance}")
    print(f"yaw_gain_near_scale      : {args.yaw_gain_near_scale}")
    print(f"yaw_gain_far_scale       : {args.yaw_gain_far_scale}")
    print(f"forward_boost_enable     : {args.forward_boost_distance_enable}")
    print(f"forward_boost_near_dist  : {args.forward_boost_near_distance}")
    print(f"forward_boost_far_dist   : {args.forward_boost_far_distance}")
    print(f"forward_boost_near_scale : {args.forward_boost_near_scale}")
    print(f"forward_boost_far_scale  : {args.forward_boost_far_scale}")
    print(f"forward_gate_distance    : {args.forward_gate_distance_enable}")
    print(f"forward_gate_near_dist   : {args.forward_gate_near_distance}")
    print(f"forward_gate_far_dist    : {args.forward_gate_far_distance}")
    print(f"forward_gate_far_mid     : {args.forward_gate_far_mid_scale}")
    print(f"forward_gate_far_min     : {args.forward_gate_far_min_scale}")
    print("")

    csv_file = None
    csv_writer = None

    if args.log_csv is not None:
        csv_file, csv_writer = open_csv_log(args.log_csv)
        print(f"CSV log                  : {args.log_csv}")
        print("")

    master = None
    sock = None
    armed_by_script = False

    command_filter = CommandFilter(args)

    latest_packet = None
    latest_receive_time = None
    latest_parse_error = None
    last_udp_seq = None

    invalid_started_at = None

    last_print_time = 0.0

    period = 1.0 / args.rate

    try:
        master = connect_mavlink(args.mavlink)

        print("")
        print("Opening UDP socket...")

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((args.udp_host, args.udp_port))
        sock.setblocking(False)

        print(f"Listening UDP on {args.udp_host}:{args.udp_port}")
        print("")

        request_mode(master, args.mode)

        print("Sending neutral MANUAL_CONTROL before arm...")
        send_stop(master, duration_s=1.5, rate_hz=args.rate, z_neutral=args.z_neutral)

        if args.arm:
            arm_vehicle(master)
            armed_by_script = True

            print("Sending neutral MANUAL_CONTROL after arm...")
            send_stop(master, duration_s=1.5, rate_hz=args.rate, z_neutral=args.z_neutral)
        else:
            print("ARM is disabled. This run will not move the vehicle.")
            print("Use --arm only after dry safety check is OK.")

        print("")
        print("Starting control loop...")
        print("Press Ctrl+C to stop early.")
        print("")

        start_time = time.time()
        last_loop_time = start_time

        while True:
            now = time.time()
            elapsed_total = now - start_time

            if elapsed_total >= args.runtime:
                break

            dt = now - last_loop_time
            last_loop_time = now

            if dt <= 0:
                dt = period

            packet, parse_error, addr = read_latest_udp_packet(sock)

            if packet is not None or parse_error is not None:
                latest_packet = packet
                latest_parse_error = parse_error
                latest_receive_time = now

                if packet is not None:
                    udp_seq_value = packet.get("udp_seq")

                    try:
                        udp_seq_int = int(udp_seq_value)
                    except (TypeError, ValueError):
                        udp_seq_int = None

                    if udp_seq_int is not None:
                        if last_udp_seq is not None:
                            seq_gap = udp_seq_int - last_udp_seq

                            if seq_gap > args.seq_jump_warning_threshold:
                                print(
                                    f"WARNING: large sequence jump. "
                                    f"last_seq={last_udp_seq}, current_seq={udp_seq_int}, gap={seq_gap}"
                                )

                        last_udp_seq = udp_seq_int

            packet_age = None

            if latest_receive_time is not None:
                packet_age = now - latest_receive_time

            hard_stop = False
            state = "NO_PACKET"
            validation_reason = "NO_PACKET"

            target_x = 0.0
            target_r = 0.0
            error_x = None
            distance_error = None

            if latest_packet is None:
                hard_stop = True
                state = "NO_PACKET"
                validation_reason = latest_parse_error or "NO_PACKET"
                invalid_started_at = None

            elif packet_age is not None and packet_age > args.packet_timeout:
                hard_stop = True
                state = "PACKET_TIMEOUT"
                validation_reason = "PACKET_TIMEOUT"
                invalid_started_at = None

            else:
                is_valid, validation_reason = validate_observation(latest_packet, args)

                if is_valid:
                    invalid_started_at = None
                    state = "TRACK"

                    target_x, target_r, error_x, distance_error = compute_target_command(
                        latest_packet,
                        args,
                        dt,
                    )
                    held_now = packet_bool(
                        latest_packet.get("held_observation", latest_packet.get("held", False))
                    )

                    if args.hard_stop_on_held and held_now:
                        target_x = 0.0
                        target_r = 0.0
                        hard_stop = True

                else:
                    if invalid_started_at is None:
                        invalid_started_at = now

                    invalid_duration = now - invalid_started_at

                    if invalid_duration <= args.invalid_decay_seconds:
                        state = "INVALID_DECAY"
                        target_x = 0.0
                        target_r = 0.0
                        hard_stop = False
                    else:
                        state = "INVALID_STOP"
                        target_x = 0.0
                        target_r = 0.0
                        hard_stop = True

            smooth_x, smooth_r = command_filter.update(
                target_x=target_x,
                target_r=target_r,
                dt=dt,
                hard_stop=hard_stop,
            )

            sent_x = round_manual_command(smooth_x)
            sent_y = 0
            sent_z = int(args.z_neutral)
            sent_r = round_manual_command(smooth_r)

            send_manual_control(
                master=master,
                x=sent_x,
                y=sent_y,
                z=sent_z,
                r=sent_r,
            )

            estimated_distance = None
            distance_confidence = None
            held_observation = None
            udp_seq = None
            packet_reason = None

            if latest_packet is not None:
                estimated_distance = latest_packet.get("estimated_distance")
                distance_confidence = latest_packet.get("distance_confidence")
                held_observation = latest_packet.get("held_observation")
                udp_seq = latest_packet.get("udp_seq")
                packet_reason = latest_packet.get("reason")

            invalid_duration_s = None

            if invalid_started_at is not None:
                invalid_duration_s = now - invalid_started_at

            write_csv_row(
                csv_writer,
                {
                    "time_unix": now,
                    "state": state,
                    "udp_seq": udp_seq,
                    "valid": latest_packet.get("valid") if latest_packet else None,
                    "reason": validation_reason,
                    "packet_age_s": packet_age,
                    "invalid_duration_s": invalid_duration_s,
                    "error_x": error_x,
                    "distance_error": distance_error,
                    "target_x": target_x,
                    "target_r": target_r,
                    "smooth_x": smooth_x,
                    "smooth_r": smooth_r,
                    "sent_x": sent_x,
                    "sent_y": sent_y,
                    "sent_z": sent_z,
                    "sent_r": sent_r,
                    "estimated_distance": estimated_distance,
                    "distance_confidence": distance_confidence,
                    "held_observation": held_observation,
                    "packet_reason": packet_reason,
                },
            )

            if now - last_print_time >= 1.0 / args.print_rate:
                last_print_time = now

                age_text = "None" if packet_age is None else f"{packet_age:.3f}s"

                print(
                    f"state={state:14s} "
                    f"seq={udp_seq} "
                    f"valid={latest_packet.get('valid') if latest_packet else None} "
                    f"held={held_observation} "
                    f"validation={validation_reason} "
                    f"packet_reason={packet_reason} "
                    f"age={age_text} "
                    f"err_x={error_x} "
                    f"dist={estimated_distance} "
                    f"target=({target_x:.1f},{target_r:.1f}) "
                    f"smooth=({smooth_x:.1f},{smooth_r:.1f}) "
                    f"cmd=({sent_x},0,{sent_z},{sent_r})"
                )

            loop_elapsed = time.time() - now
            sleep_time = period - loop_elapsed

            if sleep_time > 0:
                time.sleep(sleep_time)

        print("")
        print("Sending STOP before exit...")
        command_filter.reset()
        send_stop(master, duration_s=1.5, rate_hz=args.rate, z_neutral=args.z_neutral)

    except KeyboardInterrupt:
        print("")
        print("Interrupted by user.")
        print("Sending STOP after interrupt...")

        if master is not None:
            command_filter.reset()
            send_stop(master, duration_s=1.5, rate_hz=args.rate, z_neutral=args.z_neutral)

    finally:
        if args.arm and armed_by_script and master is not None:
            disarm_vehicle(master)

        if sock is not None:
            sock.close()

        if csv_file is not None:
            csv_file.close()

        print("Controller V2 finished safely.")


if __name__ == "__main__":
    main()
