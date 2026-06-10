import time
import threading
import sys

from mav_handler import MAVLinkHandler


DRONES = [
    {
        "name": "DRONE_1",
        "port": "COM12",
        "baud": 57600,
        "takeoff_alt": 1.0,
        "hover_time": 5,
        "start_delay": 0.0,
    },
    {
        "name": "DRONE_2",
        "port": "COM15",
        "baud": 57600,
        "takeoff_alt": 1.0,
        "hover_time": 5,
        "start_delay": 2.0,  # 2s after
    },
]

DISARM_AT_END = True



def run_offboard_test(config):
    name = config["name"]
    port = config["port"]
    baud = config["baud"]
    takeoff_alt = config["takeoff_alt"]
    hover_time = config["hover_time"]
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

        print(f"\n[{name}] Hovering at {takeoff_alt:.1f} m for {hover_time} seconds...")
        time.sleep(hover_time)

        print(f"\n[{name}] --- COMMANDING OFFBOARD DESCENT ---")

        descent_steps = [1.0, 0.8, 0.6, 0.4, 0.2, 0.0]

        for alt in descent_steps:
            mav.set_offboard_setpoint(
                mav.offboard_lat,
                mav.offboard_lon,
                alt
            )
            print(f"[{name}] Descending setpoint: {alt:.1f} m")
            time.sleep(5)

        print(f"\n[{name}] --- EXITING OFFBOARD ---")
        mav.exit_offboard()
        time.sleep(2)

        if DISARM_AT_END:
            print(f"[{name}] Disarming...")
            mav.disarm(force=True)
            time.sleep(2)

        print(f"[{name}] Flight test complete.")

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


if __name__ == "__main__":

    print("Two-drone OFFBOARD test.")
    print("Make sure:")
    print("  1. Each drone has a different COM port.")
    print("  2. Each drone has a different MAV_SYS_ID.")
    print("  3. Props are removed for the first bench test.")
    print("  4. Both drones have good GPS/EKF before flight.")
    print("")

    input("Press ENTER to start both drone threads...")

    threads = []

    for config in DRONES:
        thread = threading.Thread(
            target=run_offboard_test,
            args=(config,),
            daemon=False
        )
        threads.append(thread)
        thread.start()

    try:
        for thread in threads:
            thread.join()

    except KeyboardInterrupt:
        print("\nMain KeyboardInterrupt detected. Threads will run shutdown sequence if interrupted.")

    print("\nAll drone tests finished.")