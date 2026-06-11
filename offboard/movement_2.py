import time
import threading
import math
import sys

from mav_handler import MAVLinkHandler



DRONES = [
    {
        "name": "DRONE_1_NORTH",
        "port": "COM15",      
        "baud": 57600,
        "takeoff_alt": 1.0,
        "hover_time": 2.0,
        "move_time": 5.0,
        "north_offset_m": 2.0,   # +2 m = norte
        "east_offset_m": 0.0,
        "start_delay": 0.0,
    },
    {
        "name": "DRONE_2_SOUTH",
        "port": "COM12",      
        "baud": 57600,
        "takeoff_alt": 1.0,
        "hover_time": 2.0,
        "move_time": 5.0,
        "north_offset_m": -2.0,  # -2 m = sur
        "east_offset_m": 0.0,
        "start_delay": 2.0,
    },
]

DISARM_AT_END = True




def offset_gps(lat, lon, north_m, east_m):
    """
    Convierte offset local norte/este en lat/lon.
    Para movimientos pequeños de 1-5 m es suficientemente bueno.
    """
    d_lat = north_m / 111_320.0
    d_lon = east_m / (111_320.0 * math.cos(math.radians(lat)))
    return lat + d_lat, lon + d_lon



def run_movement_test(config):
    name = config["name"]
    port = config["port"]
    baud = config["baud"]
    takeoff_alt = config["takeoff_alt"]
    hover_time = config["hover_time"]
    move_time = config["move_time"]
    north_offset_m = config["north_offset_m"]
    east_offset_m = config["east_offset_m"]
    start_delay = config["start_delay"]

    mav = MAVLinkHandler(port=port, baud=baud)

    try:
        if start_delay > 0:
            print(f"[{name}] Waiting {start_delay:.1f} s before starting...")
            time.sleep(start_delay)

        print(f"\n[{name}] Connecting on {port}...")
        mav.connect()

        print(f"[{name}] Starting telemetry and command stream...")
        mav.start()
        time.sleep(1.0)

        print(f"[{name}] Requesting Altitude Mode...")
        mav.set_altitude_mode()
        time.sleep(0.5)

        print(f"\n[{name}] --- ARMING ---")
        armed = mav.arm()

        if not armed:
            print(f"[{name}] Failed to arm. Stopping this drone.")
            mav.stop()
            return

        time.sleep(0.5)

        print(f"\n[{name}] --- PREPARING OFFBOARD TAKEOFF ---")
        offboard_ok = mav.init_offboard(
            pre_stream_sec=3,
            takeoff_alt=takeoff_alt
        )

        if not offboard_ok:
            print(f"[{name}] Failed to enter OFFBOARD. Disarming.")
            mav.disarm(force=True)
            mav.stop()
            return

        print(f"\n[{name}] Hovering at {takeoff_alt:.1f} m for {hover_time:.1f} s...")
        time.sleep(hover_time)

        # Guardar posición inicial antes de cambiar setpoint
        home_lat = mav.offboard_lat
        home_lon = mav.offboard_lon
        home_alt = takeoff_alt

        print(f"\n[{name}] Home position:")
        print(f"[{name}]   Lat: {home_lat:.7f}")
        print(f"[{name}]   Lon: {home_lon:.7f}")
        print(f"[{name}]   Alt: {home_alt:.2f} m")

        target_lat, target_lon = offset_gps(
            home_lat,
            home_lon,
            north_m=north_offset_m,
            east_m=east_offset_m
        )

        direction = "NORTH" if north_offset_m > 0 else "SOUTH" if north_offset_m < 0 else "NO NORTH/SOUTH"
        print(f"\n[{name}] --- MOVING {abs(north_offset_m):.1f} m {direction} ---")
        print(f"[{name}] Target:")
        print(f"[{name}]   Lat: {target_lat:.7f}")
        print(f"[{name}]   Lon: {target_lon:.7f}")
        print(f"[{name}]   Alt: {home_alt:.2f} m")

        mav.set_offboard_setpoint(target_lat, target_lon, home_alt)
        time.sleep(move_time)

        print(f"\n[{name}] --- HOLDING TARGET ---")
        mav.set_offboard_setpoint(target_lat, target_lon, home_alt)
        time.sleep(2.0)


        print(f"\n[{name}] --- COMMANDING OFFBOARD DESCENT ---")

        for alt in [1.0, 0.5, 0.0]:
            mav.set_offboard_setpoint(
                mav.offboard_lat,
                mav.offboard_lon,
                alt
            )
            print(f"[{name}] Descending setpoint: {alt:.1f} m")
            time.sleep(2.0)

        print(f"\n[{name}] --- EXITING OFFBOARD ---")
        mav.exit_offboard()
        time.sleep(2.0)

        if DISARM_AT_END:
            print(f"[{name}] Disarming...")
            mav.disarm(force=True)
            time.sleep(2.0)

        print(f"[{name}] Movement test complete.")

    except KeyboardInterrupt:
        print(f"\n[{name}] KeyboardInterrupt detected.")

    except Exception as e:
        print(f"\n[{name}] ERROR: {e}")

    finally:
        print(f"[{name}] Shutdown sequence...")

        try:
            mav.virtual_pitch = 0
            mav.virtual_roll = 0
            mav.virtual_yaw = 0
            mav.virtual_throttle = 0

            if DISARM_AT_END:
                mav.disarm(force=True)

            mav.stop()

        except Exception as e:
            print(f"[{name}] Shutdown error: {e}")

        print(f"[{name}] Done.")


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    print("Two-drone GPS movement test.")
    print("")
    print("Planned movement:")
    print("  DRONE_1_NORTH -> 2 m north")
    print("  DRONE_2_SOUTH -> 2 m south")
    print("")
    print("Before running:")
    print("  1. Confirm each drone has a different COM port.")
    print("  2. Confirm each drone has a different MAV_SYS_ID.")
    print("  3. Confirm both have good GPS/EKF.")
    print("  4. First test with enough physical separation.")
    print("")

    input("Press ENTER to start both drone movement tests...")

    threads = []

    for config in DRONES:
        thread = threading.Thread(
            target=run_movement_test,
            args=(config,),
            daemon=False
        )
        threads.append(thread)
        thread.start()

    try:
        for thread in threads:
            thread.join()

    except KeyboardInterrupt:
        print("\nMain KeyboardInterrupt detected.")

    print("\nAll drone movement tests finished.")