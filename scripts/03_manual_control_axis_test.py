from pymavlink import mavutil
import time


MAVLINK_CONNECTION = "udpin:127.0.0.1:14551"

ARMED_FLAG = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED


def is_armed_from_heartbeat(msg):
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

        armed = is_armed_from_heartbeat(msg)
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
        print("WARNING: Vehicle did not arm.")

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
        print("WARNING: Vehicle did not disarm.")

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


def run_command_with_monitoring(
    master,
    target_system,
    label,
    x=0,
    y=0,
    z=500,
    r=0,
    duration=2.0,
    hz=20
):
    print("")
    print(f"=== TEST: {label} ===")
    print(f"Command: x={x}, y={y}, z={z}, r={r}, duration={duration}s")

    period = 1.0 / hz
    end_time = time.time() + duration
    next_print = time.time()

    count = 0

    while time.time() < end_time:
        # Kritik nokta: her döngüde komut göndermeye devam ediyoruz.
        send_manual_control(master, target_system, x=x, y=y, z=z, r=r)
        count += 1

        # Blocking olmayan mesaj okuma.
        msg = master.recv_match(
            type=["HEARTBEAT", "VFR_HUD", "LOCAL_POSITION_NED"],
            blocking=False
        )

        now = time.time()

        if msg is not None and now >= next_print:
            data = msg.to_dict()
            msg_type = msg.get_type()

            if msg_type == "HEARTBEAT":
                armed = (data.get("base_mode", 0) & ARMED_FLAG) != 0
                print(
                    f"HEARTBEAT armed={armed} "
                    f"base_mode={data.get('base_mode')} "
                    f"custom_mode={data.get('custom_mode')}"
                )

            elif msg_type == "VFR_HUD":
                print(
                    f"VFR_HUD throttle={data.get('throttle')} "
                    f"groundspeed={data.get('groundspeed'):.4f} "
                    f"alt={data.get('alt'):.4f} "
                    f"climb={data.get('climb'):.4f}"
                )

            elif msg_type == "LOCAL_POSITION_NED":
                print(
                    f"LOCAL_POSITION_NED "
                    f"x={data.get('x'):.4f}, "
                    f"y={data.get('y'):.4f}, "
                    f"z={data.get('z'):.4f}, "
                    f"vx={data.get('vx'):.4f}, "
                    f"vy={data.get('vy'):.4f}, "
                    f"vz={data.get('vz'):.4f}"
                )

            next_print = now + 0.5

        time.sleep(period)

    print(f"Messages sent during {label}: {count}")


def stop(master, target_system, duration=1.5):
    run_command_with_monitoring(
        master,
        target_system,
        label="STOP",
        x=0,
        y=0,
        z=500,
        r=0,
        duration=duration,
        hz=20
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

    try:
        set_mode_manual(master, target_system)

        # Arm öncesi nötr komut
        stop(master, target_system, duration=1.0)

        armed_ok = arm_vehicle(master, target_system, target_component)

        if not armed_ok:
            print("Axis test cancelled because vehicle did not arm.")
            return

        # Armed olduktan sonra önce kısa stop
        stop(master, target_system, duration=2.0)

        # Düşük komutlarla kısa eksen testleri
        run_command_with_monitoring(
            master, target_system,
            label="FORWARD_X_POSITIVE",
            x=250, y=0, z=500, r=0,
            duration=2.0,
            hz=20
        )
        stop(master, target_system, duration=1.5)

        run_command_with_monitoring(
            master, target_system,
            label="BACKWARD_X_NEGATIVE",
            x=-250, y=0, z=500, r=0,
            duration=2.0,
            hz=20
        )
        stop(master, target_system, duration=1.5)

        run_command_with_monitoring(
            master, target_system,
            label="YAW_R_POSITIVE",
            x=0, y=0, z=500, r=250,
            duration=2.0,
            hz=20
        )
        stop(master, target_system, duration=1.5)

        run_command_with_monitoring(
            master, target_system,
            label="YAW_R_NEGATIVE",
            x=0, y=0, z=500, r=-250,
            duration=2.0,
            hz=20
        )
        stop(master, target_system, duration=2.0)

    finally:
        disarm_vehicle(master, target_system, target_component)

        # Disarm sonrası kısa nötr
        stop(master, target_system, duration=1.0)

    print("Manual control axis test finished.")


if __name__ == "__main__":
    main()
