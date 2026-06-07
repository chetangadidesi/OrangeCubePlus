import sys
import time
import math
from pymavlink import mavutil

from mav_handler import MAVLinkHandler


# ==========================
# USER CONFIG
# ==========================

PORT = "COM12"
BAUD = 57600

TAKEOFF_ALT_M = 1.0       
NORTH_OFFSET_M = 1.0      
EAST_OFFSET_M = 0.0

HOVER_BEFORE_MOVE_SEC = 3
MOVE_TIMEOUT_SEC = 10
HOLD_AT_TARGET_SEC = 3

ARRIVAL_RADIUS_M = 0.6    # For 1 m move, 0.6 m is realistic with GPS noise

USE_LAND_MODE = True


# ==========================
# GEOMETRY HELPERS
# ==========================

def offset_lat_lon(lat_deg, lon_deg, north_m, east_m):
    """
    Convert local north/east offset in meters to GPS lat/lon.
    Good enough for very small movements like 1 m.
    """
    lat_rad = math.radians(lat_deg)

    target_lat = lat_deg + north_m / 111111.0
    target_lon = lon_deg + east_m / (111111.0 * math.cos(lat_rad))

    return target_lat, target_lon


def distance_meters(lat1, lon1, lat2, lon2):
    """
    Approximate horizontal distance between two GPS coordinates.
    """
    lat_avg = math.radians((lat1 + lat2) / 2.0)

    north = (lat2 - lat1) * 111111.0
    east = (lon2 - lon1) * 111111.0 * math.cos(lat_avg)

    return math.sqrt(north ** 2 + east ** 2)


def wait_for_global_position(mav, timeout_sec=10):
    """
    Wait until the background thread has current GLOBAL_POSITION_INT data.
    Returns lat, lon, rel_alt.
    """
    start = time.time()

    while time.time() - start < timeout_sec:
        if mav.global_lat is not None and mav.global_lon is not None and mav.global_alt is not None:
            return mav.global_lat, mav.global_lon, mav.global_alt

        time.sleep(0.1)

    return None


def set_land_mode(mav):
    """
    Request PX4 AUTO.LAND mode.
    This is safer than trying to descend all the way to 0 using manual offboard altitude setpoints.
    """
    print("Requesting LAND mode...")

    mav.offboard_active = False
    time.sleep(0.2)

    mav.connection.mav.command_long_send(
        mav.connection.target_system,
        mav.connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        1,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        4,  # PX4_CUSTOM_MAIN_MODE_AUTO
        6,  # PX4_CUSTOM_SUB_MODE_AUTO_LAND
        0, 0, 0, 0
    )


def print_current_position(mav, label="Current position"):
    pos = wait_for_global_position(mav, timeout_sec=2)

    if pos is None:
        print(f"{label}: No GLOBAL_POSITION_INT available")
        return None

    lat, lon, alt = pos
    print(f"{label}:")
    print(f"  Lat: {lat:.7f}")
    print(f"  Lon: {lon:.7f}")
    print(f"  RelAlt: {alt:.2f} m")

    return pos


# ==========================
# MAIN
# ==========================

if __name__ == "__main__":

    mav = MAVLinkHandler(port=PORT, baud=BAUD)

    try:
        print("Connecting...")
        mav.connect()

        print("Starting telemetry and command stream...")
        mav.start()
        time.sleep(1.0)

        print("Requesting Altitude Mode...")
        mav.set_altitude_mode()
        time.sleep(0.5)

        print("\n--- ARMING ---")
        armed = mav.arm()

        if not armed:
            print("Failed to arm. Shutting down.")
            mav.stop()
            sys.exit(1)

        time.sleep(0.5)

        print("\n--- ENTERING OFFBOARD TAKEOFF/HOLD ---")
        print(f"Requested takeoff altitude: {TAKEOFF_ALT_M:.2f} m")

        offboard_ok = mav.init_offboard(
            pre_stream_sec=3,
            takeoff_alt=TAKEOFF_ALT_M
        )

        if not offboard_ok:
            print("Failed to enter OFFBOARD. Shutting down.")
            mav.disarm(force=True)
            mav.stop()
            sys.exit(1)

        print("\n--- HOVER BEFORE MOVE ---")
        for i in range(HOVER_BEFORE_MOVE_SEC):
            print_current_position(mav, label=f"Hover position t={i+1}s")
            time.sleep(1.0)

        # Read position A after takeoff/hover
        pos_a = wait_for_global_position(mav, timeout_sec=10)

        if pos_a is None:
            raise RuntimeError("Could not read GLOBAL_POSITION_INT after takeoff.")

        current_lat, current_lon, current_alt = pos_a

        print("\nPosition A:")
        print(f"  Lat: {current_lat:.7f}")
        print(f"  Lon: {current_lon:.7f}")
        print(f"  RelAlt: {current_alt:.2f} m")

        # Use current altitude as the target altitude for horizontal movement
        target_alt = current_alt

        target_lat, target_lon = offset_lat_lon(
            current_lat,
            current_lon,
            NORTH_OFFSET_M,
            EAST_OFFSET_M
        )

        print("\nTarget B:")
        print(f"  North offset: {NORTH_OFFSET_M:.2f} m")
        print(f"  East offset: {EAST_OFFSET_M:.2f} m")
        print(f"  Target Lat: {target_lat:.7f}")
        print(f"  Target Lon: {target_lon:.7f}")
        print(f"  Target RelAlt: {target_alt:.2f} m")

        initial_dist = distance_meters(
            current_lat,
            current_lon,
            target_lat,
            target_lon
        )

        print(f"  Initial distance to B: {initial_dist:.2f} m")

        input("\nPress ENTER to command movement to B = A + 1 m north...")

        print("\n--- MOVING TO TARGET B ---")
        mav.set_offboard_setpoint(target_lat, target_lon, target_alt)

        start_move = time.time()
        reached = False

        while time.time() - start_move < MOVE_TIMEOUT_SEC:
            pos = wait_for_global_position(mav, timeout_sec=2)

            if pos is None:
                print("No GLOBAL_POSITION_INT during move.")
                time.sleep(0.5)
                continue

            lat, lon, alt = pos
            dist = distance_meters(lat, lon, target_lat, target_lon)

            print(
                f"Distance to B: {dist:.2f} m | "
                f"Lat: {lat:.7f}, Lon: {lon:.7f}, RelAlt: {alt:.2f} m"
            )

            if dist <= ARRIVAL_RADIUS_M:
                print("Reached B approximately.")
                reached = True
                break

            time.sleep(1.0)

        if not reached:
            print("Move timeout reached. Holding current target anyway.")

        print("\n--- HOLDING TARGET B ---")
        mav.set_offboard_setpoint(target_lat, target_lon, target_alt)

        for i in range(HOLD_AT_TARGET_SEC):
            print_current_position(mav, label=f"Hold B t={i+1}s")
            time.sleep(1.0)

        print("\n--- LANDING / SHUTDOWN ---")

        if USE_LAND_MODE:
            set_land_mode(mav)
            time.sleep(10)
        else:
            # Backup option: do not prefer this yet.
            print("Manual offboard descent disabled by default.")

        print("Sending disarm...")
        mav.disarm(force=True)
        time.sleep(1.0)

        mav.stop()
        print("Done.")

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Emergency shutdown.")

        try:
            set_land_mode(mav)
            time.sleep(5)
            mav.disarm(force=True)
            mav.stop()
        except Exception as e:
            print(f"Emergency shutdown error: {e}")

    except Exception as e:
        print(f"\nERROR: {e}")

        try:
            set_land_mode(mav)
            time.sleep(5)
            mav.disarm(force=True)
            mav.stop()
        except Exception as shutdown_error:
            print(f"Shutdown error: {shutdown_error}")