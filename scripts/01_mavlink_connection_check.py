from pymavlink import mavutil
import time


MAVLINK_CONNECTION = "udpin:127.0.0.1:14551"


def main():
    print(f"Connecting to MAVLink: {MAVLINK_CONNECTION}")

    master = mavutil.mavlink_connection(MAVLINK_CONNECTION)

    print("Waiting for heartbeat...")
    heartbeat = master.wait_heartbeat(timeout=30)

    if heartbeat is None:
        print("ERROR: No heartbeat received within timeout.")
        return

    print("Heartbeat received.")
    print(f"target_system    = {master.target_system}")
    print(f"target_component = {master.target_component}")

    print("\nListening for a few MAVLink messages...")
    start = time.time()

    while time.time() - start < 5:
        msg = master.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue

        msg_type = msg.get_type()

        if msg_type in ["HEARTBEAT", "SYS_STATUS", "ATTITUDE", "VFR_HUD", "LOCAL_POSITION_NED"]:
            print(msg_type, msg.to_dict())

    print("\nMAVLink connection check finished.")


if __name__ == "__main__":
    main()
