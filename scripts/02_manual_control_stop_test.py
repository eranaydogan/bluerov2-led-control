from pymavlink import mavutil
import time


MAVLINK_CONNECTION = "udpin:127.0.0.1:14551"

NEUTRAL_X = 0
NEUTRAL_Y = 0
NEUTRAL_Z = 500
NEUTRAL_R = 0

ARMED_FLAG = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED


def is_heartbeat_armed(msg):
    if msg is None:
        return False

    if msg.get_type() != "HEARTBEAT":
        return False

    base_mode = msg.to_dict().get("base_mode", 0)
    return (base_mode & ARMED_FLAG) != 0


def wait_arm_state(master, desired_armed, timeout=10):
    """
    pymavlink motors_armed_wait(timeout=...) bazı sürümlerde çalışmadığı için
    armed/disarmed durumunu HEARTBEAT base_mode içinden kendimiz kontrol ediyoruz.
    """
    start = time.time()

    while time.time() - start < timeout:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)

        if msg is None:
            continue

        armed = is_heartbeat_armed(msg)
        base_mode = msg.to_dict().get("base_mode", None)
        custom_mode = msg.to_dict().get("custom_mode", None)
        system_status = msg.to_dict().get("system_status", None)

        print(
            f"HEARTBEAT: armed={armed} "
            f"base_mode={base_mode} "
            f"custom_mode={custom_mode} "
            f"system_status={system_status}"
        )

        if armed == desired_armed:
            return True

    return False


def send_neutral_manual_control(master, target_system, duration=5.0, hz=20):
    period = 1.0 / hz
    end_time = time.time() + duration

    print(
        f"Sending neutral MANUAL_CONTROL for {duration} seconds "
        f"at {hz} Hz: x=0, y=0, z=500, r=0"
    )

    count = 0

    while time.time() < end_time:
        master.mav.manual_control_send(
            target_system,
            NEUTRAL_X,
            NEUTRAL_Y,
            NEUTRAL_Z,
            NEUTRAL_R,
            0
        )
        count += 1
        time.sleep(period)

    print(f"Neutral MANUAL_CONTROL messages sent: {count}")


def set_mode_manual(master, target_system):
    mode_mapping = master.mode_mapping()

    print("Available modes:", list(mode_mapping.keys()))

    if "MANUAL" not in mode_mapping:
        raise RuntimeError("MANUAL mode not found in mode mapping.")

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
        1,  # 1 = arm
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
        print("WARNING: Vehicle did not report ARMED state within timeout.")

    return ok


def disarm_vehicle(master, target_system, target_component):
    print("Sending DISARM command...")

    master.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,  # 0 = disarm
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
        print("WARNING: Vehicle did not report DISARMED state within timeout.")

    return ok


def print_status_messages(master, duration=3.0):
    print(f"Reading status messages for {duration} seconds...")
    start = time.time()

    while time.time() - start < duration:
        msg = master.recv_match(
            type=["HEARTBEAT", "VFR_HUD", "LOCAL_POSITION_NED"],
            blocking=True,
            timeout=1
        )

        if msg is None:
            continue

        msg_type = msg.get_type()
        data = msg.to_dict()

        if msg_type == "HEARTBEAT":
            armed = (data.get("base_mode", 0) & ARMED_FLAG) != 0
            print(
                f"HEARTBEAT: armed={armed} "
                f"base_mode={data.get('base_mode')} "
                f"custom_mode={data.get('custom_mode')} "
                f"system_status={data.get('system_status')}"
            )

        elif msg_type == "VFR_HUD":
            print(
                f"VFR_HUD: throttle={data.get('throttle')} "
                f"groundspeed={data.get('groundspeed'):.4f} "
                f"alt={data.get('alt'):.4f} "
                f"climb={data.get('climb'):.4f}"
            )

        elif msg_type == "LOCAL_POSITION_NED":
            print(
                f"LOCAL_POSITION_NED: "
                f"x={data.get('x'):.4f}, "
                f"y={data.get('y'):.4f}, "
                f"z={data.get('z'):.4f}, "
                f"vx={data.get('vx'):.4f}, "
                f"vy={data.get('vy'):.4f}, "
                f"vz={data.get('vz'):.4f}"
            )


def main():
    print(f"Connecting to MAVLink: {MAVLINK_CONNECTION}")

    master = mavutil.mavlink_connection(MAVLINK_CONNECTION)

    print("Waiting for heartbeat...")
    heartbeat = master.wait_heartbeat(timeout=30)

    if heartbeat is None:
        print("ERROR: No heartbeat received.")
        return

    target_system = master.target_system
    target_component = master.target_component

    print("Heartbeat received.")
    print(f"target_system    = {target_system}")
    print(f"target_component = {target_component}")

    armed_ok = False

    try:
        print_status_messages(master, duration=2.0)

        set_mode_manual(master, target_system)

        # Arm etmeden önce nötr manual control gönderiyoruz.
        send_neutral_manual_control(master, target_system, duration=2.0, hz=20)

        armed_ok = arm_vehicle(master, target_system, target_component)

        if armed_ok:
            # Armed durumdayken sadece nötr komut gönderiyoruz.
            send_neutral_manual_control(master, target_system, duration=5.0, hz=20)
            print_status_messages(master, duration=3.0)
        else:
            print("Skipping armed neutral test because vehicle did not arm.")

    finally:
        # Test sonunda güvenli şekilde disarm deniyoruz.
        disarm_vehicle(master, target_system, target_component)

        # Disarm sonrası kısa nötr komut.
        send_neutral_manual_control(master, target_system, duration=1.0, hz=20)

    print("Manual control STOP test finished.")


if __name__ == "__main__":
    main()
