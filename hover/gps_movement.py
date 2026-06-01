import sys
import time
import math
import threading
from pymavlink import mavutil


from mav_handler import MAVLinkHandler


PORT = "COM13"
BAUD = 57600

HOVER_THROTTLE = 500
TAKEOFF_THROTTLE = 700
TAKEOFF_TIME = 1.0
HOVER_BEFORE_GPS_TIME = 2.0

TARGET_REL_ALT_M = 2.0
NORTH_OFFSET_M = 1.0
EAST_OFFSET_M = 0.0

SETPOINT_RATE_HZ = 10
SETPOINT_DURATION_SEC = 15
ARRIVAL_RADIUS_M = 0.8

DISARM_AT_END = True



def offset_lat_lon(lat_deg, lon_deg, north_m, east_m):
    """
    Convert local north/east meter offset into lat/lon.
    Good enough for small movements like 1-5 m.
    """
    lat_rad = math.radians(lat_deg)

    target_lat = lat_deg + north_m / 111111.0
    target_lon = lon_deg + east_m / (111111.0 * math.cos(lat_rad))

    return target_lat, target_lon


def distance_meters(lat1, lon1, lat2, lon2):
    """
    Approximate distance between two GPS points in meters.
    """
    lat_avg = math.radians((lat1 + lat2) / 2.0)

    north = (lat2 - lat1) * 111111.0
    east = (lon2 - lon1) * 111111.0 * math.cos(lat_avg)

    return math.sqrt(north**2 + east**2)


def wait_for_gps_fix(connection, min_fix_type=3):
    """
    Wait until GPS_RAW_INT reports at least 3D fix.
    fix_type:
      0/1 = no fix
      2 = 2D
      3 = 3D
      4 = DGPS
      5 = RTK float
      6 = RTK fixed
    """
    print("Waiting for GPS fix...")

    while True:
        msg = connection.recv_match(type="GPS_RAW_INT", blocking=True, timeout=5)

        if msg is None:
            print("No GPS_RAW_INT message yet...")
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


def read_current_global_position(connection):
    """
    Prefer GLOBAL_POSITION_INT because it includes relative_alt.
    Returns:
      lat, lon, relative_alt_m
    """
    msg = connection.recv_match(
        type="GLOBAL_POSITION_INT",
        blocking=True,
        timeout=2
    )

    if msg is None:
        return None

    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    rel_alt = msg.relative_alt / 1000.0

    return lat, lon, rel_alt


def send_global_position_setpoint(connection, lat_deg, lon_deg, rel_alt_m):
    """
    Send one PX4 global position setpoint.
    Must be called repeatedly for OFFBOARD.
    """
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

    connection.mav.set_position_target_global_int_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        type_mask,
        lat_int,
        lon_int,
        rel_alt_m,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )


def set_offboard_mode(connection):
    """
    Switch PX4 to OFFBOARD mode.
    This assumes setpoints are already being sent.
    """
    mode_mapping = connection.mode_mapping()
    print("Available modes:", mode_mapping)

    if mode_mapping is None or "OFFBOARD" not in mode_mapping:
        print("OFFBOARD mode not available.")
        return False

    offboard_mode = mode_mapping["OFFBOARD"]

    # PX4 usually returns a tuple: (base_mode, main_mode, sub_mode)
    if isinstance(offboard_mode, tuple):
        base_mode, main_mode, sub_mode = offboard_mode

        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            base_mode,
            main_mode,
            sub_mode,
            0, 0, 0, 0
        )
    else:
        # Fallback for other mappings
        connection.set_mode(offboard_mode)

    ack = connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    print(f"OFFBOARD ACK: {ack}")

    return True


def disarm_drone(connection):
    print("Disarming...")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,  # disarm
        0, 0, 0, 0, 0, 0
    )

    ack = connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    print(f"Disarm ACK: {ack}")


def start_status_listener(connection):
    """
    Prints PX4 STATUSTEXT messages in the background.
    Useful if OFFBOARD is rejected or failsafe triggers.
    """
    def _listen():
        while True:
            msg = connection.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
            if msg is not None:
                print(f"\nPX4 STATUS: {msg.text}")

    thread = threading.Thread(target=_listen, daemon=True)
    thread.start()



if __name__ == "__main__":

    mav = MAVLinkHandler(port=PORT, baud=BAUD)

    try:
        print("Connecting using same style as main(4).py...")
        mav.connect()

        connection = mav.connection
        start_status_listener(connection)

        print("Starting telemetry and virtual joystick stream...")
        mav.start()
        time.sleep(1.0)

        print("Setting Altitude Mode...")
        mav.set_altitude_mode()
        time.sleep(1.0)

        print("Waiting for good GPS before arming...")
        gps_lat, gps_lon, gps_alt = wait_for_gps_fix(connection, min_fix_type=3)

        print("Pre-loading throttle to hover value...")
        mav.virtual_roll = 0
        mav.virtual_pitch = 0
        mav.virtual_yaw = 0
        mav.virtual_throttle = HOVER_THROTTLE
        time.sleep(1.0)

        print("\nREADY FOR GPS SMALL MOVE TEST.")
        print("This test will:")
        print(f"  1. Arm")
        print(f"  2. Takeoff/hover using virtual throttle")
        print(f"  3. Read current GPS position A")
        print(f"  4. Move to B = A + {NORTH_OFFSET_M} m north, {EAST_OFFSET_M} m east")
        print(f"  5. Hold target for up to {SETPOINT_DURATION_SEC} seconds")
        print("")
        input("Press ENTER to ARM and start...")

        print("Arming...")
        mav.arm()
        time.sleep(0.7)

        print("\n--- TAKEOFF BLIP ---")
        mav.virtual_roll = 0
        mav.virtual_pitch = 0
        mav.virtual_yaw = 0
        mav.virtual_throttle = TAKEOFF_THROTTLE
        time.sleep(TAKEOFF_TIME)

        print("--- HOVER BEFORE GPS MOVE ---")
        mav.virtual_throttle = HOVER_THROTTLE
        mav.virtual_pitch = 0
        time.sleep(HOVER_BEFORE_GPS_TIME)

        print("\nReading current global position A...")
        current_pos = None

        for _ in range(10):
            current_pos = read_current_global_position(connection)
            if current_pos is not None:
                break
            print("No GLOBAL_POSITION_INT yet...")
            time.sleep(0.2)

        if current_pos is None:
            raise RuntimeError("Could not read GLOBAL_POSITION_INT. Aborting GPS move.")

        current_lat, current_lon, current_rel_alt = current_pos

        print("Current position A:")
        print(f"  Lat: {current_lat:.7f}")
        print(f"  Lon: {current_lon:.7f}")
        print(f"  Relative Alt: {current_rel_alt:.2f} m")

        if current_rel_alt > 0.5:
            target_rel_alt = current_rel_alt
        else:
            target_rel_alt = TARGET_REL_ALT_M

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

        print("Target position B:")
        print(f"  Lat: {target_lat:.7f}")
        print(f"  Lon: {target_lon:.7f}")
        print(f"  Relative Alt: {target_rel_alt:.2f} m")
        print(f"  Approx distance: {initial_dist:.2f} m")


        print("\nSending initial GPS setpoints before OFFBOARD...")
        for _ in range(30):  # 3 seconds at 10 Hz
            send_global_position_setpoint(
                connection,
                target_lat,
                target_lon,
                target_rel_alt
            )
            time.sleep(1.0 / SETPOINT_RATE_HZ)


        print("Switching to OFFBOARD...")
        offboard_ok = set_offboard_mode(connection)

        if not offboard_ok:
            raise RuntimeError("Could not switch to OFFBOARD. Aborting GPS move.")

        time.sleep(0.5)

        # --------------------------------------------------
        # 6. Continue sending target B
        # --------------------------------------------------
        print("\n--- MOVING TO GPS TARGET B ---")
        dt = 1.0 / SETPOINT_RATE_HZ
        total_steps = int(SETPOINT_DURATION_SEC * SETPOINT_RATE_HZ)

        reached = False

        for i in range(total_steps):
            send_global_position_setpoint(
                connection,
                target_lat,
                target_lon,
                target_rel_alt
            )

            if i % SETPOINT_RATE_HZ == 0:
                pos = read_current_global_position(connection)

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
                    print("No current global position.")

            time.sleep(dt)

        print("\n--- HOLDING GPS TARGET ---")
        for _ in range(30):  # 3 seconds
            send_global_position_setpoint(
                connection,
                target_lat,
                target_lon,
                target_rel_alt
            )
            time.sleep(1.0 / SETPOINT_RATE_HZ)

        print("GPS small move test complete.")

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Stopping.")

    except Exception as e:
        print(f"\nERROR: {e}")

    finally:
        print("\nShutdown sequence...")
        try:
            # Stop virtual joystick inputs
            mav.virtual_pitch = 0
            mav.virtual_roll = 0
            mav.virtual_yaw = 0
            mav.virtual_throttle = 0
            time.sleep(0.5)

            if DISARM_AT_END:
                disarm_drone(mav.connection)

            mav.stop()

        except Exception as e:
            print(f"Shutdown error: {e}")

        print("Done.")