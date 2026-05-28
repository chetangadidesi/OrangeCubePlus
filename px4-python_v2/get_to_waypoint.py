import time
import math
from pymavlink import mavutil


PORT = "COM12"     # Drone COMM
BAUD = 57600

TARGET_REL_ALT_M = 3.0   # Z goal
NORTH_OFFSET_M = 3.0     # waypoint 3m north
EAST_OFFSET_M = 0.0      # Waypoint 0m east

SETPOINT_DURATION_SEC = 30
SETPOINT_RATE_HZ = 10
ARRIVAL_RADIUS_M = 0.8
ARM_MOTORS = True
DISARM_AT_END = True


def connect_drone():
    drone = mavutil.mavlink_connection(PORT, baud=BAUD)

    print("Waiting for heartbeat...")
    drone.wait_heartbeat()
    print(f"Connected! System: {drone.target_system}, Component: {drone.target_component}")

    return drone


def read_gps(drone):
    while True:
        msg = drone.recv_match(type="GPS_RAW_INT", blocking=True, timeout=5)

        if msg is None:
            print("No GPS message yet...")
            continue

        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1000.0
        fix_type = msg.fix_type
        sats = msg.satellites_visible

        print(
            f"GPS → Fix: {fix_type}, Sats: {sats}, "
            f"Lat: {lat}, Lon: {lon}, Alt: {alt:.2f} m"
        )

        if fix_type >= 3:
            return lat, lon, alt

        print("Waiting for 3D GPS fix...")


def offset_lat_lon(lat_deg, lon_deg, north_m, east_m):
    lat_rad = math.radians(lat_deg)

    target_lat = lat_deg + north_m / 111111.0
    target_lon = lon_deg + east_m / (111111.0 * math.cos(lat_rad))

    return target_lat, target_lon

def distance_meters(lat1, lon1, lat2, lon2):
    """
    Approximate distance between two GPS points in meters.
    Good enough for small offsets like 3 m.
    """
    lat_avg = math.radians((lat1 + lat2) / 2.0)

    north = (lat2 - lat1) * 111111.0
    east = (lon2 - lon1) * 111111.0 * math.cos(lat_avg)

    return math.sqrt(north**2 + east**2)


def read_current_position(drone):
    """
    Reads current estimated global position.
    Prefer GLOBAL_POSITION_INT because it includes relative_alt.
    Falls back to GPS_RAW_INT if needed.
    """
    msg = drone.recv_match(
        type=["GLOBAL_POSITION_INT", "GPS_RAW_INT"],
        blocking=True,
        timeout=1
    )

    if msg is None:
        return None

    msg_type = msg.get_type()

    if msg_type == "GLOBAL_POSITION_INT":
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        rel_alt = msg.relative_alt / 1000.0
        return lat, lon, rel_alt

    if msg_type == "GPS_RAW_INT":
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1000.0
        return lat, lon, alt

    return None


def set_mode_if_available(drone):
    """
    PX4 uses OFFBOARD mode for external setpoint control.

    In pymavlink, PX4 mode_mapping returns:
    OFFBOARD: (base_mode, main_mode, sub_mode)
    Example:
    OFFBOARD: (29, 6, 0)
    """
    mode_mapping = drone.mode_mapping()
    print("Available modes:", mode_mapping)

    if mode_mapping is None:
        print("Could not read mode mapping.")
        return False

    if "OFFBOARD" not in mode_mapping:
        print("OFFBOARD mode not found. Cannot switch to PX4 OFFBOARD.")
        return False

    base_mode, main_mode, sub_mode = mode_mapping["OFFBOARD"]

    drone.mav.command_long_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        base_mode,   # param1: base mode
        main_mode,   # param2: PX4 custom main mode
        sub_mode,    # param3: PX4 custom sub mode
        0, 0, 0, 0
    )

    ack = drone.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    print(f"OFFBOARD ACK: {ack}")

    time.sleep(2)
    return True

def print_status_text(drone, duration_sec=5):
    """
    Prints PX4 status/error messages for a few seconds.
    Useful to understand why arming was rejected.
    """
    print("Reading PX4 status messages...")
    start = time.time()

    while time.time() - start < duration_sec:
        msg = drone.recv_match(type="STATUSTEXT", blocking=True, timeout=1)

        if msg is not None:
            print(f"PX4 STATUS: {msg.text}")


def arm_drone(drone):
    print("Arming motors...")
    drone.mav.command_long_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,  # 1 = arm
        0, 0, 0, 0, 0, 0
    )

    ack = drone.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    print(f"Arm ACK: {ack}")

    if ack is None or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Arm was not accepted. Checking PX4 reason...")
        print_status_text(drone, duration_sec=5)
        return False

    return True


def disarm_drone(drone):
    print("Disarming motors...")
    drone.mav.command_long_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,  # 0 = disarm
        0, 0, 0, 0, 0, 0
    )

    ack = drone.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    print(f"Disarm ACK: {ack}")

def send_waypoint_setpoint(drone, lat_deg, lon_deg, rel_alt_m):
    lat_int = int(lat_deg * 1e7)
    lon_int = int(lon_deg * 1e7)

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

    print("Sending waypoint/setpoint continuously:")
    print(f"  Target Lat: {lat_deg}")
    print(f"  Target Lon: {lon_deg}")
    print(f"  Target Relative Alt: {rel_alt_m} m")

    dt = 1.0 / SETPOINT_RATE_HZ
    total_steps = int(SETPOINT_DURATION_SEC * SETPOINT_RATE_HZ)

    for i in range(total_steps):
        drone.mav.set_position_target_global_int_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            drone.target_system,
            drone.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            lat_int,
            lon_int,
            rel_alt_m,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )

        if i % SETPOINT_RATE_HZ == 0:
            pos = read_current_position(drone)

            if pos is not None:
                current_lat, current_lon, current_alt = pos
                dist = distance_meters(current_lat, current_lon, lat_deg, lon_deg)

                print(
                    f"Distance to waypoint: {dist:.2f} m | "
                    f"Current Lat: {current_lat:.7f}, Lon: {current_lon:.7f}, Alt: {current_alt:.2f} m"
                )

                if dist <= ARRIVAL_RADIUS_M:
                    print("Waypoint reached approximately.")
                    break
            else:
                print("No current position message received.")

        time.sleep(dt)


if __name__ == "__main__":
    drone = connect_drone()

    current_lat, current_lon, current_alt = read_gps(drone)

    target_lat, target_lon = offset_lat_lon(
        current_lat,
        current_lon,
        NORTH_OFFSET_M,
        EAST_OFFSET_M
    )

    initial_dist = distance_meters(
        current_lat,
        current_lon,
        target_lat,
        target_lon
    )

    print("Current position:")
    print(f"  Lat: {current_lat}")
    print(f"  Lon: {current_lon}")
    print(f"  GPS Alt: {current_alt:.2f} m")

    print("Target waypoint:")
    print(f"  Lat: {target_lat}")
    print(f"  Lon: {target_lon}")
    print(f"  Relative Alt: {TARGET_REL_ALT_M:.2f} m")
    print(f"  Approx distance from start: {initial_dist:.2f} m")

    # Start sending setpoints before/around mode switch.
    # This helps especially for PX4 OFFBOARD behavior.
    print("Sending initial setpoints before mode switch...")
    for _ in range(20):
        lat_int = int(target_lat * 1e7)
        lon_int = int(target_lon * 1e7)

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

        drone.mav.set_position_target_global_int_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            drone.target_system,
            drone.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            lat_int,
            lon_int,
            TARGET_REL_ALT_M,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        time.sleep(0.1)

    if ARM_MOTORS:
        armed = arm_drone(drone)
        time.sleep(2)

        if not armed:
            print("Stopping because vehicle did not arm.")
            exit()

    set_mode_if_available(drone)

    try:
        send_waypoint_setpoint(
            drone,
            target_lat,
            target_lon,
            TARGET_REL_ALT_M
        )
    finally:
        if DISARM_AT_END:
            disarm_drone(drone)

    print("Done.")