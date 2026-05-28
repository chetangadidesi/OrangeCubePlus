import time
import math
from pymavlink import mavutil


PORT = "COM8"     # Drone COMM
BAUD = 57600

TARGET_REL_ALT_M = 3.0   # Z goal
NORTH_OFFSET_M = 3.0     # waypoint 3m north
EAST_OFFSET_M = 0.0      # Waypoint 0m east


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

    print(f"Sending waypoint:")
    print(f"  Lat: {lat_deg}")
    print(f"  Lon: {lon_deg}")
    print(f"  Relative Alt: {rel_alt_m} m")

    for _ in range(50):  # send for 5 seconds at 10 Hz
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
        time.sleep(0.1)


if __name__ == "__main__":
    drone = connect_drone()

    current_lat, current_lon, current_alt = read_gps(drone)

    target_lat, target_lon = offset_lat_lon(
        current_lat,
        current_lon,
        NORTH_OFFSET_M,
        EAST_OFFSET_M
    )

    print("Current position:")
    print(f"  Lat: {current_lat}")
    print(f"  Lon: {current_lon}")
    print(f"  GPS Alt: {current_alt:.2f} m")

    send_waypoint_setpoint(
        drone,
        target_lat,
        target_lon,
        TARGET_REL_ALT_M
    )

    print("Done.")