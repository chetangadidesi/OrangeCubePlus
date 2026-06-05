import sys
import time
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

from mav_handler import MAVLinkHandler
from pymavlink import mavutil

# ------------------------------------------------------------------ #
#  GUI — shown after flight for data review                           #
# ------------------------------------------------------------------ #

class IMUVisualizer(QtWidgets.QMainWindow):
    def __init__(self, mav_handler):
        super().__init__()
        self.mav_handler = mav_handler

        self.setWindowTitle("Cube Orange+ (PX4) — Post-flight Review")
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QtWidgets.QVBoxLayout(self.central_widget)

        # --- PLOT 1: ACCELEROMETER ---
        self.accel_plot = pg.PlotWidget(title="Accelerometer (m/s²)")
        self.layout.addWidget(self.accel_plot)
        self.accel_plot.addLegend()
        self.accel_plot.setYRange(-15, 15)
        self.curve_ax = self.accel_plot.plot(pen='r', name="X (Front)")
        self.curve_ay = self.accel_plot.plot(pen='g', name="Y (Right)")
        self.curve_az = self.accel_plot.plot(pen='b', name="Z (Down)")

        # --- PLOT 2: Z-GYRO ---
        self.gyro_plot = pg.PlotWidget(title="Z-Gyro (rad/s)")
        self.layout.addWidget(self.gyro_plot)
        self.gyro_plot.addLegend()
        self.gyro_plot.setYRange(-10, 10)
        self.curve_gz = self.gyro_plot.plot(pen='y', name="Yaw Rate")

        # --- PLOT 3: ATTITUDE ---
        self.att_plot = pg.PlotWidget(title="Fused Attitude (Degrees)")
        self.layout.addWidget(self.att_plot)
        self.att_plot.addLegend()
        self.att_plot.setYRange(0, 360)
        self.curve_pitch = self.att_plot.plot(pen=(255, 165, 0), name="Pitch")
        self.curve_roll  = self.att_plot.plot(pen=(255, 0, 255),  name="Roll")
        self.curve_yaw   = self.att_plot.plot(pen=(0, 255, 255),  name="Yaw")

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(30)

    def update_plot(self):
        if len(self.mav_handler.data_ax) > 0:
            self.curve_ax.setData(list(self.mav_handler.data_ax))
            self.curve_ay.setData(list(self.mav_handler.data_ay))
            self.curve_az.setData(list(self.mav_handler.data_az))
            self.curve_gz.setData(list(self.mav_handler.data_gz))
        if len(self.mav_handler.data_pitch) > 0:
            self.curve_pitch.setData(list(self.mav_handler.data_pitch))
            self.curve_roll.setData(list(self.mav_handler.data_roll))
            self.curve_yaw.setData(list(self.mav_handler.data_yaw))

    def closeEvent(self, event):
        self.mav_handler.stop()
        super().closeEvent(event)


# ------------------------------------------------------------------ #
#  Main flight script                                                 #
# ------------------------------------------------------------------ #

# if __name__ == "__main__":

#     mav = MAVLinkHandler(port='COM13', baud=57600)
#     mav.connect()

#     # ── 1. Start telemetry ────────────────────────────────────────────
#     print("Starting telemetry and command stream...")
#     mav.start()
#     time.sleep(1)

#     # ── 2. Wait for EKF / GPS BEFORE Arming ───────────────────────────
#     print("\n--- WAITING FOR EKF ALIGNMENT ---")
#     mav.wait_for_gps_fix(min_fix_type=3)
#     pos = mav.read_global_position()
#     if pos is None:
#         print("EKF never aligned. Shutting down.")
#         mav.stop()
#         sys.exit()

#     # ── 3. Set Altitude Hold mode and pre-load throttle ───────────────
#     mav.set_altitude_mode()
#     mav.virtual_throttle = 500
#     time.sleep(1)

#     # ── 4. Arm & Fly ──────────────────────────────────────────────────
#     mav.arm()
#     time.sleep(0.5)
#     # ================================================================ #
#     #  PHASE 1 — Altitude-Hold flight (manual-control joystick)        #
#     # ================================================================ #

#     print("\n--- TAKEOFF BLIP ---")
#     mav.virtual_throttle = 0
#     time.sleep(1.5)

#     print("--- HOVER ---")
#     mav.virtual_throttle = 0
#     time.sleep(3)

#     # print("--- YAW RIGHT ---")
#     # mav.virtual_yaw = -300
#     # time.sleep(3)

#     # print("--- STOP YAW ---")
#     # mav.virtual_yaw = 0
#     # time.sleep(3)

#     # print("--- HOVER ---")
#     # mav.virtual_throttle = 500
#     # time.sleep(3)

#     # ================================================================ #
#     #  PHASE 2 — OFFBOARD position-hold                                #
#     # ================================================================ #
#     #
#     #  init_offboard() does the full sequence automatically:
#     #    • waits for a 3-D GPS fix
#     #    • reads the current fused position
#     #    • pre-streams setpoints for 3 s  (required by PX4)
#     #    • sends MAV_CMD_DO_SET_MODE → OFFBOARD
#     #    • enables continuous setpoint streaming in the background thread
#     #
#     #  If your environment has no GPS (bench test), comment out this
#     #  entire block and skip straight to the SHUTDOWN section.
#     # ---------------------------------------------------------------- #

#     print("\n--- ENTERING OFFBOARD MODE ---")
    
#     offboard_ok = mav.init_offboard(pre_stream_sec=3)
#     print(offboard_ok)


#     if offboard_ok:
#         print("Holding current GPS position for 10 seconds...")
#         time.sleep(10)

#         # ── Optional: command a new GPS waypoint ──────────────────────
#         # new_lat = mav.offboard_lat + 0.00005   # ~5 m north
#         # mav.set_offboard_setpoint(new_lat, mav.offboard_lon, mav.offboard_alt)
#         # time.sleep(5)

#         print("--- EXITING OFFBOARD ---")
#         mav.exit_offboard()
#         time.sleep(2)
#     else:
#         print("OFFBOARD not working")
#         print("\n--- SHUTTING DOWN ---")
#         mav.virtual_throttle = 0
#         time.sleep(0.5)
#         mav.disarm(force=True)
#         mav.stop()

#     # ================================================================ #
#     #  SHUTDOWN                                                         #
#     # ================================================================ #

#     print("\n--- SHUTTING DOWN ---")
#     mav.virtual_throttle = 0
#     time.sleep(0.5)

#     mav.disarm(force=True)
#     time.sleep(1)

#     mav.stop()
#     print("Drone safely disarmed. Launching GUI for data review...")

#     # ── Post-flight GUI ───────────────────────────────────────────────
#     app = QtWidgets.QApplication(sys.argv)
#     window = IMUVisualizer(mav)
#     window.show()
#     sys.exit(app.exec_())


if __name__ == "__main__":

    mav = MAVLinkHandler(port='COM13', baud=57600)
    mav.connect()

    # ── 1. Start telemetry ────────────────────────────────────────────
    print("Starting telemetry and command stream...")
    mav.start()
    time.sleep(1)

    print("Requesting Altitude Mode...")
    mav.set_altitude_mode()
    time.sleep(0.5)
    
    # ── 2. ARM THE DRONE ──────────────────────────────────────────────
    print("\n--- ARMING ---")
    armed = mav.arm()
    if not armed:
        print("Failed to arm. Shutting down.")
        mav.stop()
        sys.exit()

    time.sleep(0.5)


    # ── 4. Initialize OFFBOARD (Target: 2 meters up) ──────────────────
    print("\n--- PREPARING OFFBOARD AUTONOMOUS TAKEOFF ---")
    
    # Now that the FC thinks it's flying, it will gladly accept the mode switch!
    offboard_ok = mav.init_offboard(pre_stream_sec=3, takeoff_alt=1.0)

    if not offboard_ok:
        print("Failed to enter offboard. Shutting down.")
        mav.disarm(force=True)
        mav.stop()
        sys.exit()

    # ── 5. HOVER ──────────────────────────────────────────────────────
    print("\nHovering at 2 meters for 10 seconds...")
    time.sleep(3)

    # ── 5. AUTO-LAND ──────────────────────────────────────────────────
    print("\n--- COMMANDING OFFBOARD DESCENT ---")
    mav.set_offboard_setpoint(mav.offboard_lat, mav.offboard_lon, 0.0)
    time.sleep(5) 

    # ── 6. SHUTDOWN ───────────────────────────────────────────────────
    print("\n--- SHUTTING DOWN ---")
    mav.exit_offboard()
    time.sleep(0.5)

    mav.disarm(force=True)
    time.sleep(1)

    mav.stop()
    print("Drone safely disarmed. Launching GUI for data review...")

    # ── Post-flight GUI ───────────────────────────────────────────────
    app = QtWidgets.QApplication(sys.argv)
    window = IMUVisualizer(mav)
    window.show()
    sys.exit(app.exec_())
