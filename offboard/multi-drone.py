"""
main_multi_drone.py
====================
Flies multiple drones simultaneously using per-drone threads.
Each drone gets its own waypoint sequence while sharing the same
safety scaffold from old_main.py (sleeps, arm checks, descent steps,
disarm, stop).

Configure each drone in the DRONES list below.
"""

import sys
import time
import math
import threading

from mav_handler import MAVLinkHandler


# ------------------------------------------------------------------ #
#  Thread-safe logging                                                #
# ------------------------------------------------------------------ #

_print_lock = threading.Lock()

def log(name, msg):
    with _print_lock:
        print(f"[{name}] {msg}")


# ------------------------------------------------------------------ #
#  GPS offset helper                                                  #
# ------------------------------------------------------------------ #

def offset_gps(lat, lon, north_m, east_m):
    """Return a new (lat, lon) offset by north_m / east_m metres."""
    d_lat = north_m / 111_320.0
    d_lon = east_m  / (111_320.0 * math.cos(math.radians(lat)))
    return lat + d_lat, lon + d_lon


# ------------------------------------------------------------------ #
#  Per-drone configuration                                            #
# ------------------------------------------------------------------ #
#
#  waypoints  — list of (north_m, east_m, alt_m) tuples relative to
#               the drone's own home position captured at takeoff.
#               The drone visits them in order, hovers at each for
#               `wp_hover_time` seconds, then returns home.
#
#  start_delay — seconds to wait before this drone begins its sequence
#               (lets you stagger takeoffs so they don't interfere).
#
# ------------------------------------------------------------------ #

DRONES = [
    {
        "name":          "DRONE_1",
        "port":          "COM12",
        "baud":          57600,
        "takeoff_alt":   1.0,        # metres AGL
        "hover_time":    3,          # seconds to hover after takeoff before WPs
        "wp_hover_time": 5,          # seconds to hover at each waypoint
        "start_delay":   0.0,        # seconds before starting
        "waypoints": [               # (north_m, east_m, alt_m) relative to home
            (-2.0,  0.0, 1.0),       # 2 m south
            ( 0.0,  2.0, 1.0),       # 2 m east
        ],
    },
    {
        "name":          "DRONE_2",
        "port":          "COM15",
        "baud":          57600,
        "takeoff_alt":   1.0,
        "hover_time":    3,
        "wp_hover_time": 5,
        "start_delay":   2.0,        # start 2 s after DRONE_1
        "waypoints": [
            ( 2.0,  0.0, 1.0),       # 2 m north
            ( 0.0, -2.0, 1.0),       # 2 m west
        ],
    },
]

DISARM_AT_END = True


# ------------------------------------------------------------------ #
#  Per-drone flight routine                                           #
# ------------------------------------------------------------------ #

def run_drone(config):
    name          = config["name"]
    port          = config["port"]
    baud          = config["baud"]
    takeoff_alt   = config["takeoff_alt"]
    hover_time    = config["hover_time"]
    wp_hover_time = config["wp_hover_time"]
    start_delay   = config["start_delay"]
    waypoints     = config["waypoints"]

    mav = MAVLinkHandler(port=port, baud=baud)

    # Tracks whether we got airborne — the finally block uses this to
    # decide if a landing command is needed or not.
    is_airborne = False
    mav_started = False

    try:
        # ── 0. Optional staggered start ───────────────────────────────
        if start_delay > 0:
            log(name, f"Waiting {start_delay:.1f} s before starting...")
            time.sleep(start_delay)

        # ── 1. Connect & start telemetry ──────────────────────────────
        log(name, f"Connecting on {port}...")
        mav.connect()

        log(name, "Starting telemetry and command stream...")
        mav.start()
        mav_started = True
        time.sleep(2)

        # ── 2. Set flight mode ────────────────────────────────────────
        log(name, "Requesting Altitude Mode...")
        mav.set_altitude_mode()
        time.sleep(0.5)

        # ── 3. Arm ────────────────────────────────────────────────────
        log(name, "--- ARMING ---")
        armed = mav.arm()
        if not armed:
            log(name, "Failed to arm. Shutting down this drone.")
            return   # finally will skip landing since is_airborne=False

        time.sleep(1)

        # ── 4. Offboard takeoff ───────────────────────────────────────
        log(name, "--- PREPARING OFFBOARD AUTONOMOUS TAKEOFF ---")
        offboard_ok = mav.init_offboard(pre_stream_sec=3, takeoff_alt=takeoff_alt)

        if not offboard_ok:
            log(name, "Failed to enter OFFBOARD. Disarming.")
            mav.disarm(force=True)
            return   # finally will skip landing since is_airborne=False

        is_airborne = True   # we are now in the air

        # ── 5. Initial hover ──────────────────────────────────────────
        log(name, f"Hovering at {takeoff_alt:.1f} m for {hover_time} s...")
        time.sleep(hover_time)

        # Save home position AFTER takeoff so it reflects the live EKF fix
        home_lat = mav.offboard_lat
        home_lon = mav.offboard_lon
        home_alt = takeoff_alt
        log(name, f"Home locked → Lat: {home_lat:.7f}, Lon: {home_lon:.7f}")

        # ── 6. Visit waypoints ────────────────────────────────────────
        for idx, (north_m, east_m, wp_alt) in enumerate(waypoints, start=1):
            wp_lat, wp_lon = offset_gps(home_lat, home_lon, north_m, east_m)
            log(name, f"--- WAYPOINT {idx}/{len(waypoints)}: "
                      f"N{north_m:+.1f} m  E{east_m:+.1f} m  alt {wp_alt:.1f} m ---")
            mav.set_offboard_setpoint(wp_lat, wp_lon, wp_alt)
            time.sleep(wp_hover_time)

        # ── 7. Return home ────────────────────────────────────────────
        log(name, "--- RETURNING HOME ---")
        mav.set_offboard_setpoint(home_lat, home_lon, home_alt)
        time.sleep(5)

        # ── 8. Offboard descent ───────────────────────────────────────
        # Build steps dynamically from takeoff_alt down to 0
        log(name, "--- COMMANDING OFFBOARD DESCENT ---")
        step_size  = 0.2
        num_steps  = int(takeoff_alt / step_size)
        descent_steps = [round(takeoff_alt - i * step_size, 1) for i in range(num_steps + 1)]

        for alt in descent_steps:
            mav.set_offboard_setpoint(mav.offboard_lat, mav.offboard_lon, alt)
            log(name, f"Descending setpoint: {alt:.1f} m")
            time.sleep(2)

        # ── 9. Exit offboard ──────────────────────────────────────────
        log(name, "--- EXITING OFFBOARD ---")
        mav.exit_offboard()
        time.sleep(2)

        # ── 10. Land mode ─────────────────────────────────────────────
        log(name, "--- COMMANDING LAND MODE ---")
        mav.land()
        time.sleep(15)          # wait for drone to fully settle on ground

        is_airborne = False     # drone is on the ground now

        # ── 11. Disarm ────────────────────────────────────────────────
        if DISARM_AT_END:
            log(name, "Disarming...")
            mav.disarm(force=True)
            time.sleep(2)

        log(name, "Flight complete.")

    except KeyboardInterrupt:
        log(name, "KeyboardInterrupt detected — running shutdown sequence.")

    except Exception as e:
        log(name, f"ERROR: {e}")

    finally:
        # ── Safety net ────────────────────────────────────────────────
        # Only runs emergency landing if we were actually airborne
        # (avoids calling land() on a drone that never took off or is
        # already on the ground after a clean flight).
        log(name, "Shutdown sequence...")
        try:
            if is_airborne:
                log(name, "Drone is airborne — commanding LAND mode for safe descent...")
                mav.offboard_active = False
                time.sleep(0.1)
                mav.land()
                log(name, "Waiting 15 s for landing to complete...")
                time.sleep(15)

                mav.virtual_pitch    = 0
                mav.virtual_roll     = 0
                mav.virtual_yaw      = 0
                mav.virtual_throttle = 0

                if DISARM_AT_END:
                    log(name, "Disarming after emergency landing...")
                    mav.disarm(force=True)
                    time.sleep(2)

            if mav_started:
                mav.stop()

        except Exception as e:
            log(name, f"Shutdown error: {e}")

        log(name, "Done.")


# ------------------------------------------------------------------ #
#  Entry point                                                        #
# ------------------------------------------------------------------ #

if __name__ == "__main__":

    print("=" * 60)
    print("  Multi-drone OFFBOARD flight")
    print("=" * 60)
    print("Pre-flight checklist:")
    print("  1. Each drone is on a unique COM port.")
    print("  2. Each drone has a unique MAV_SYS_ID.")
    print("  3. Props checked.")
    print("  4. Both drones have good GPS/EKF before flight.")
    print()
    print("Loaded drone configs:")
    for cfg in DRONES:
        print(f"  {cfg['name']}  port={cfg['port']}  "
              f"takeoff_alt={cfg['takeoff_alt']} m  "
              f"waypoints={len(cfg['waypoints'])}  "
              f"start_delay={cfg['start_delay']} s")
    print()

    input("Press ENTER to start all drone threads...")

    threads = []
    for config in DRONES:
        t = threading.Thread(
            target=run_drone,
            args=(config,),
            name=config["name"],
            daemon=False,   # keep process alive until all threads finish
        )
        threads.append(t)
        t.start()

    try:
        for t in threads:
            t.join()

    except KeyboardInterrupt:
        print("\nMain thread interrupted. "
              "Drone threads will finish their shutdown sequences.")

    print("\nAll drones finished.")
