import time
from pymavlink import mavutil


PORT = "COM12"
BAUD = 57600

SETPOINT_RATE_HZ = 10
PRE_OFFBOARD_SEC = 3
HOLD_SEC = 6


GPS_RAW_INT_ID = 24
GLOBAL_POSITION_INT_ID = 33


def connect():
    print("Connecting raw MAVLink...")
    master = mavutil.mavlink_connection(PORT, baud=BAUD)

    print("Waiting for heartbeat...")
    master.wait_heartbeat(timeout=20)

    print("Connected!")
    print(f"System: {master.target_system}, Component: {master.target_component}")
    return master


def request_message_interval(master, message_id, hz):
    interval_us = int(1e6 / hz)

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
        0, 0, 0, 0, 0
    )

    print(f"Requested message ID {message_id} at {hz} Hz")


def wait_for_gps_fix(master, min_fix_type=3):
    print("Waiting for GPS fix...")

    while True:
        msg = master.recv_match(type="GPS_RAW_INT", blocking=True, timeout=5)

        if msg is None:
            print("No GPS_RAW_INT yet...")
            continue

        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1000.0
        fix_type = msg.fix_type
        sats = msg.satellites_visible

        print(
            f"GPS → Fix: {fix_type}, Sats: {sats}, "
            f"Lat: {lat:.7f}, Lon: {lon:.7f}, Alt: {alt:.2f} m"
        )

        if fix_type >= min_fix_type:
            return lat, lon, alt


def read_global_position(master, attempts=30):
    print("Reading GLOBAL_POSITION_INT...")

    for _ in range(attempts):
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)

        if msg is not None:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            rel_alt = msg.relative_alt / 1000.0

            print(
                f"GLOBAL_POSITION_INT → "
                f"Lat: {lat:.7f}, Lon: {lon:.7f}, RelAlt: {rel_alt:.2f} m"
            )

            return lat, lon, rel_alt

        print("No GLOBAL_POSITION_INT yet...")

    return None


def send_global_position_setpoint(master, lat, lon, rel_alt):
    lat_int = int(lat * 1e7)
    lon_int = int(lon * 1e7)

    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    master.mav.set_position_target_global_int_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        type_mask,
        lat_int,
        lon_int,
        rel_alt,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )


def set_offboard_mode(master):
    mode_mapping = master.mode_mapping()
    print("Available modes:", mode_mapping)

    if mode_mapping is None or "OFFBOARD" not in mode_mapping:
        print("OFFBOARD not available.")
        return False

    base_mode, main_mode, sub_mode = mode_mapping["OFFBOARD"]

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        base_mode,
        main_mode,
        sub_mode,
        0, 0, 0, 0
    )

    ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    print(f"OFFBOARD ACK: {ack}")

    if ack is None:
        return False

    if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print(f"OFFBOARD rejected. Result: {ack.result}")
        return False

    return True


def print_heartbeat_state(master, label):
    msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=3)

    if msg is None:
        print(f"{label}: No heartbeat.")
        return

    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    print(f"{label}:")
    print(f"  base_mode: {msg.base_mode}")
    print(f"  custom_mode: {msg.custom_mode}")
    print(f"  armed: {armed}")
    print(f"  system_status: {msg.system_status}")


def print_status_messages(master, seconds=3):
    print("Reading PX4 status messages...")
    start = time.time()

    while time.time() - start < seconds:
        msg = master.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
        if msg is not None:
            print(f"PX4 STATUS: {msg.text}")


if __name__ == "__main__":
    master = None

    try:
        master = connect()

        request_message_interval(master, GPS_RAW_INT_ID, 5)
        request_message_interval(master, GLOBAL_POSITION_INT_ID, 10)

        time.sleep(1)

        print_heartbeat_state(master, "Initial state")

        wait_for_gps_fix(master, min_fix_type=3)

        pos = read_global_position(master)

        if pos is None:
            raise RuntimeError("Could not get GLOBAL_POSITION_INT.")

        lat, lon, rel_alt = pos

        print("\nUsing current position as OFFBOARD setpoint:")
        print(f"  Lat: {lat:.7f}")
        print(f"  Lon: {lon:.7f}")
        print(f"  RelAlt: {rel_alt:.2f} m")

        input("\nPress ENTER to send setpoints and request OFFBOARD. Motors will NOT arm...")

        dt = 1.0 / SETPOINT_RATE_HZ

        print(f"\nSending setpoints for {PRE_OFFBOARD_SEC} seconds...")
        for _ in range(int(PRE_OFFBOARD_SEC * SETPOINT_RATE_HZ)):
            send_global_position_setpoint(master, lat, lon, rel_alt)
            time.sleep(dt)

        print("\nRequesting OFFBOARD...")
        ok = set_offboard_mode(master)

        if not ok:
            print("OFFBOARD failed.")
            print_status_messages(master, seconds=5)
            raise RuntimeError("OFFBOARD not accepted.")

        print("OFFBOARD accepted.")

        print(f"\nHolding same setpoint for {HOLD_SEC} seconds...")
        for i in range(int(HOLD_SEC * SETPOINT_RATE_HZ)):
            send_global_position_setpoint(master, lat, lon, rel_alt)

            if i % SETPOINT_RATE_HZ == 0:
                print_heartbeat_state(master, "Current state")

            time.sleep(dt)

        print("\nTest complete. No motors were armed.")

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt. Exiting.")

    except Exception as e:
        print(f"\nERROR: {e}")

    finally:
        print("\nDone.")