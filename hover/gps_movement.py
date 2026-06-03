import time
import math
from pymavlink import mavutil


# ============================================================
# CONFIGURATION
# ============================================================

PORT = "COM12"
BAUD = 57600

TARGET_REL_ALT_M = 1.0      # First test: low altitude
NORTH_OFFSET_M = 0.5        # First test: small movement north
EAST_OFFSET_M = 0.0

SETPOINT_RATE_HZ = 10
PRE_OFFBOARD_SEC = 3
MOVE_TIMEOUT_SEC = 15
HOLD_AT_TARGET_SEC = 10

ARRIVAL_RADIUS_M = 0.2

ARM_MOTORS = True
DISARM_AT_END = False       # Important: do NOT disarm automatically in the air

GPS_RAW_INT_ID = 24
GLOBAL_POSITION_INT_ID = 33

AUTO_LAND_AT_END = True
AUTO_DISARM_AFTER_LAND = True
LAND_TIMEOUT_SEC = 25
LANDED_ALT_THRESHOLD_M = 0.20


# ============================================================
# BASIC MAVLINK HELPERS
# ============================================================

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

        print("GPS fix not good enough yet. Waiting...")


def read_global_position(master, attempts=30):
    for _ in range(attempts):
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)

        if msg is not None:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            rel_alt = msg.relative_alt / 1000.0

            return lat, lon, rel_alt

        print("No GLOBAL_POSITION_INT yet...")

    return None


def print_heartbeat_state(master, label):
    msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=3)

    if msg is None:
        print(f"{label}: No heartbeat.")
        return None

    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    print(f"{label}:")
    print(f"  base_mode: {msg.base_mode}")
    print(f"  custom_mode: {msg.custom_mode}")
    print(f"  armed: {armed}")
    print(f"  system_status: {msg.system_status}")

    return armed


def print_status_messages(master, seconds=3):
    print("Reading PX4 status messages...")
    start = time.time()

    while time.time() - start < seconds:
        msg = master.recv_match(type="STATUSTEXT", blocking=True, timeout=1)

        if msg is not None:
            print(f"PX4 STATUS: {msg.text}")


# ============================================================
# GEOMETRY
# ============================================================

def offset_lat_lon(lat_deg, lon_deg, north_m, east_m):
    """
    Convert local north/east offset in meters to lat/lon.
    Good enough for small movements.
    """
    lat_rad = math.radians(lat_deg)

    target_lat = lat_deg + north_m / 111111.0
    target_lon = lon_deg + east_m / (111111.0 * math.cos(lat_rad))

    return target_lat, target_lon


def distance_meters(lat1, lon1, lat2, lon2):
    """
    Approximate distance between two GPS points in meters.
    Good enough for small offsets.
    """
    lat_avg = math.radians((lat1 + lat2) / 2.0)

    north = (lat2 - lat1) * 111111.0
    east = (lon2 - lon1) * 111111.0 * math.cos(lat_avg)

    return math.sqrt(north ** 2 + east ** 2)


# ============================================================
# OFFBOARD SETPOINTS
# ============================================================

def send_global_position_setpoint(master, lat, lon, rel_alt):
    """
    Send one PX4 global position setpoint.
    Must be called repeatedly for OFFBOARD.
    """
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
        print("OFFBOARD failed: no ACK received.")
        return False

    if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print(f"OFFBOARD rejected. Result: {ack.result}")
        return False

    print("OFFBOARD accepted.")
    return True


# ============================================================
# ARM / DISARM / LAND
# ============================================================

def arm_drone(master):
    print("Arming motors...")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,  # arm
        0, 0, 0, 0, 0, 0
    )

    ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    print(f"ARM ACK: {ack}")

    if ack is None:
        print("Arm failed: no ACK received.")
        print_status_messages(master, seconds=5)
        return False

    if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print(f"Arm rejected. Result: {ack.result}")
        print_status_messages(master, seconds=5)
        return False

    print("Arm accepted.")
    return True


def disarm_drone(master):
    print("Disarming motors...")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,  # disarm
        0, 0, 0, 0, 0, 0
    )

    ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    print(f"DISARM ACK: {ack}")


def set_land_mode(master):
    """
    Safer than disarm for ending a real flight.
    This only requests LAND mode. It does not force-disarm.
    """
    mode_mapping = master.mode_mapping()

    if mode_mapping is None or "LAND" not in mode_mapping:
        print("LAND mode not available.")
        return False

    base_mode, main_mode, sub_mode = mode_mapping["LAND"]

    print("Requesting LAND mode...")

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
    print(f"LAND ACK: {ack}")

    if ack is None:
        return False

    return ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED


def land_and_disarm(master):
    print("\n--- AUTO LAND SEQUENCE ---")

    land_ok = set_land_mode(master)

    if not land_ok:
        print("LAND mode was not accepted. Keeping vehicle armed; manual intervention needed.")
        return False

    print("LAND mode accepted. Waiting for vehicle to reach ground...")

    start = time.time()

    while time.time() - start < LAND_TIMEOUT_SEC:
        pos = read_global_position(master, attempts=1)

        if pos is not None:
            lat, lon, rel_alt = pos
            print(f"Landing... Relative Alt: {rel_alt:.2f} m")

            if rel_alt <= LANDED_ALT_THRESHOLD_M:
                print("Altitude is low enough. Waiting 2 seconds before disarm...")
                time.sleep(2.0)

                if AUTO_DISARM_AFTER_LAND:
                    disarm_drone(master)

                return True

        time.sleep(0.5)

    print("Landing timeout reached. Not forcing disarm.")
    return False
# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":
    master = None

    try:
        master = connect()

        request_message_interval(master, GPS_RAW_INT_ID, 5)
        request_message_interval(master, GLOBAL_POSITION_INT_ID, 10)

        time.sleep(1)

        print_heartbeat_state(master, "Initial state")

        wait_for_gps_fix(master, min_fix_type=3)

        print("\nReading current global position A...")
        current_pos = read_global_position(master)

        if current_pos is None:
            raise RuntimeError("Could not get GLOBAL_POSITION_INT.")

        current_lat, current_lon, current_rel_alt = current_pos

        print("Current position A:")
        print(f"  Lat: {current_lat:.7f}")
        print(f"  Lon: {current_lon:.7f}")
        print(f"  Relative Alt: {current_rel_alt:.2f} m")

        target_lat, target_lon = offset_lat_lon(
            current_lat,
            current_lon,
            NORTH_OFFSET_M,
            EAST_OFFSET_M
        )

        target_rel_alt = TARGET_REL_ALT_M

        initial_dist = distance_meters(
            current_lat,
            current_lon,
            target_lat,
            target_lon
        )

        print("\nTarget position B:")
        print(f"  Lat: {target_lat:.7f}")
        print(f"  Lon: {target_lon:.7f}")
        print(f"  Relative Alt: {target_rel_alt:.2f} m")
        print(f"  Horizontal distance: {initial_dist:.2f} m")

        print("\nSAFETY CHECK:")
        print("  - Propellers installed correctly only if you are ready for real flight.")
        print("  - Drone must be outside, level, with clear area.")
        print("  - QGC/RC should be ready to switch to Hold/Land.")
        print("  - This script will NOT auto-disarm at the end.")
        print("  - Target movement is intentionally small.")

        input("\nPress ENTER to start GPS movement test...")

        dt = 1.0 / SETPOINT_RATE_HZ

        # ----------------------------------------------------
        # 1. Send setpoints BEFORE OFFBOARD
        # ----------------------------------------------------
        print(f"\nSending target setpoints for {PRE_OFFBOARD_SEC} seconds before OFFBOARD...")

        for _ in range(int(PRE_OFFBOARD_SEC * SETPOINT_RATE_HZ)):
            send_global_position_setpoint(master, target_lat, target_lon, target_rel_alt)
            time.sleep(dt)

        # ----------------------------------------------------
        # 2. Arm first
        if ARM_MOTORS:
            print("\nArming before OFFBOARD...")
            armed_ok = arm_drone(master)

            if not armed_ok:
                raise RuntimeError("Vehicle did not arm. Aborting safely.")

            time.sleep(1.0)

        # Keep sending setpoints after arming and before OFFBOARD
        print("\nSending more setpoints after arming...")
        for _ in range(int(2 * SETPOINT_RATE_HZ)):
            send_global_position_setpoint(master, target_lat, target_lon, target_rel_alt)
            time.sleep(dt)

        # 3. Switch to OFFBOARD
        print("\nRequesting OFFBOARD mode...")
        offboard_ok = set_offboard_mode(master)

        if not offboard_ok:
            raise RuntimeError("OFFBOARD was not accepted. Aborting safely.")

        time.sleep(0.5)

        # ----------------------------------------------------
        # 4. Continue sending target B
        # ----------------------------------------------------
        print("\n--- MOVING TO TARGET B ---")

        reached = False
        total_steps = int(MOVE_TIMEOUT_SEC * SETPOINT_RATE_HZ)

        for i in range(total_steps):
            send_global_position_setpoint(master, target_lat, target_lon, target_rel_alt)

            if i % SETPOINT_RATE_HZ == 0:
                pos = read_global_position(master, attempts=1)

                if pos is not None:
                    lat, lon, rel_alt = pos
                    dist = distance_meters(lat, lon, target_lat, target_lon)

                    print(
                        f"Distance to B: {dist:.2f} m | "
                        f"Lat: {lat:.7f}, Lon: {lon:.7f}, Alt: {rel_alt:.2f} m"
                    )

                    if dist <= ARRIVAL_RADIUS_M:
                        print("Reached B approximately.")
                        reached = True
                        break
                else:
                    print("No GLOBAL_POSITION_INT during movement.")

            time.sleep(dt)

        # ----------------------------------------------------
        # 5. Hold target B
        # ----------------------------------------------------
        print(f"\n--- HOLDING TARGET B FOR {HOLD_AT_TARGET_SEC} SEC ---")

        for i in range(int(HOLD_AT_TARGET_SEC * SETPOINT_RATE_HZ)):
            send_global_position_setpoint(master, target_lat, target_lon, target_rel_alt)

            if i % SETPOINT_RATE_HZ == 0:
                pos = read_global_position(master, attempts=1)

                if pos is not None:
                    lat, lon, rel_alt = pos
                    dist = distance_meters(lat, lon, target_lat, target_lon)

                    print(
                        f"Holding B | Dist: {dist:.2f} m | "
                        f"Alt: {rel_alt:.2f} m"
                    )

            time.sleep(dt)

        print("\nGPS movement test complete.")

        if AUTO_LAND_AT_END:
            land_and_disarm(master)
        else:
            print("\nAUTO_LAND_AT_END is False.")
            print("Vehicle will keep receiving no more setpoints after script ends.")

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected.")

        if master is not None:
            print("Requesting LAND mode because test was interrupted...")
            set_land_mode(master)

    except Exception as e:
        print(f"\nERROR: {e}")

        if master is not None:
            print("Reading status messages after error...")
            print_status_messages(master, seconds=5)

    finally:
        print("\nShutdown sequence...")

        if master is not None:
            if DISARM_AT_END:
                disarm_drone(master)
            else:
                print("DISARM_AT_END is False. No direct disarm in finally.")

        print("Done.")