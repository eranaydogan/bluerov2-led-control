import socket
import json
import time


UDP_HOST = "0.0.0.0"
UDP_PORT = 5005

DESIRED_DISTANCE = 3.0

PATTERN_ACC_MIN = 0.95
DIST_CONF_MIN = 0.60

# 03_manual_control_axis_test sonucuna göre doğrulanan işaretler:
# x > 0  -> robot baktığı yöne ileri gider
# x < 0  -> robot geri gider
# r > 0  -> robot sağa yaw yapar
# r < 0  -> robot sola yaw yapar

K_YAW = 500
K_FORWARD = 400

MAX_X = 300
MAX_R = 300


def clamp(value, low, high):
    return max(low, min(high, value))


def packet_to_command(pkt):
    valid = pkt.get("valid", False)
    face_id = pkt.get("face_id", None)

    pattern_accuracy = float(pkt.get("pattern_accuracy", 0.0))
    distance_confidence = float(pkt.get("distance_confidence", 0.0))

    if not valid:
        return 0, 0, 500, 0, "INVALID"

    if face_id != "BACK":
        return 0, 0, 500, 0, "WRONG_FACE"

    if pattern_accuracy < PATTERN_ACC_MIN:
        return 0, 0, 500, 0, "LOW_PATTERN_CONF"

    error_norm = pkt.get("error_norm", [0.0, 0.0])
    estimated_distance = pkt.get("estimated_distance", None)

    error_x = float(error_norm[0])

    # Test sonucu:
    # error_x > 0 hedef sağda demek.
    # r > 0 sağa yaw yaptığı için doğrudan çarpıyoruz.
    r = clamp(K_YAW * error_x, -MAX_R, MAX_R)

    # İlk testte vertical kapalı.
    y = 0
    z = 500

    if estimated_distance is None:
        x = 0
        return int(x), int(y), int(z), int(r), "ALIGN_ONLY_NO_DISTANCE"

    if distance_confidence < DIST_CONF_MIN:
        x = 0
        return int(x), int(y), int(z), int(r), "ALIGN_ONLY_LOW_DISTANCE_CONF"

    distance_error = float(estimated_distance) - DESIRED_DISTANCE

    # Test sonucu:
    # distance_error > 0 hedef uzakta demek.
    # x > 0 ileri gittiği için doğrudan çarpıyoruz.
    x = clamp(K_FORWARD * distance_error, -MAX_X, MAX_X)

    return int(x), int(y), int(z), int(r), "TRACK_DRY_RUN"


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_HOST, UDP_PORT))

    print(f"Listening UDP on {UDP_HOST}:{UDP_PORT}")
    print("DRY-RUN: MAVLink command will NOT be sent.")
    print(f"desired_distance={DESIRED_DISTANCE}")
    print(f"K_FORWARD={K_FORWARD}, K_YAW={K_YAW}")
    print("Vertical control disabled: z=500 fixed")
    print("")

    last_seq = None

    while True:
        data, addr = sock.recvfrom(8192)
        now = time.time()

        try:
            pkt = json.loads(data.decode("utf-8"))
        except Exception as e:
            print(f"JSON parse error from {addr}: {e}")
            continue

        seq = pkt.get("udp_seq", None)
        sent_time = pkt.get("sent_time_unix", None)

        latency_ms = None
        if sent_time is not None:
            latency_ms = (now - float(sent_time)) * 1000.0

        if last_seq is not None and seq is not None:
            if seq != last_seq + 1:
                print(f"WARNING: sequence jump. last_seq={last_seq}, current_seq={seq}")

        if seq is not None:
            last_seq = seq

        x, y, z, r, state = packet_to_command(pkt)

        error_norm = pkt.get("error_norm", None)
        estimated_distance = pkt.get("estimated_distance", None)

        latency_text = "None" if latency_ms is None else f"{latency_ms:.2f}"

        print(
            f"seq={seq} "
            f"state={state} "
            f"valid={pkt.get('valid')} "
            f"face={pkt.get('face_id')} "
            f"pattern_acc={pkt.get('pattern_accuracy')} "
            f"dist_conf={pkt.get('distance_confidence')} "
            f"err={error_norm} "
            f"dist={estimated_distance} "
            f"cmd(x,y,z,r)=({x},{y},{z},{r}) "
            f"latency_ms={latency_text}"
        )


if __name__ == "__main__":
    main()
