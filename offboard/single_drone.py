import sys
import time
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import math

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


if __name__ == "__main__":

    mav = MAVLinkHandler(port='COM13', baud=57600)
    mav.connect()


    # ── 1. Start telemetry ────────────────────────────────────────────
    print("Starting telemetry and command stream...")
    mav.start()
    time.sleep(2)
    mav.set_max_velocity(1.0)   # cap all axes to 1 m/s
    #mav.wait_for_ekf_ready()
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

    time.sleep(1)


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
    print("\nHovering at 1 meters for 3 seconds...")
    time.sleep(3)
    
    # Save home BEFORE any set_offboard_setpoint call overwrites it
    home_lat = mav.offboard_lat
    home_lon = mav.offboard_lon

    def offset_gps(lat, lon, north_m, east_m):
        d_lat = north_m / 111_320.0
        d_lon = east_m  / (111_320.0 * math.cos(math.radians(lat)))
        return lat + d_lat, lon + d_lon

    print("\n--- FLYING 5 m NORTH ---")
    #new_lat, new_lon = offset_gps(home_lat, home_lon, north_m=-2, east_m=0)
    #mav.set_offboard_setpoint(new_lat, new_lon, 1.0)
    mav.set_offboard_setpoint(29.71661614, -95.32868633, 2.0)
    time.sleep(5)
    mav.set_offboard_setpoint(29.71653087, -95.32871888, 2.0)
    time.sleep(5)
    mav.set_offboard_setpoint(29.71644561, -95.32875143, 2.0)
    
    time.sleep(5)

    print("\n--- RETURNING HOME ---")
    mav.set_offboard_setpoint(home_lat, home_lon, 1.0)  # use saved home
    time.sleep(5)

    # ── 5. AUTO-LAND ──────────────────────────────────────────────────
    print("\n--- COMMANDING OFFBOARD DESCENT ---")

    for alt in [1.0, 0.5, 0.0]:
        mav.set_offboard_setpoint(mav.offboard_lat, mav.offboard_lon, alt)
        print(f"Descending setpoint: {alt:.1f} m")
        time.sleep(2)

    # ── 6. SHUTDOWN ───────────────────────────────────────────────────
    print("\n--- SHUTTING DOWN ---")
    mav.exit_offboard()
    mav.land()
    time.sleep(20)

    mav.disarm(force=True)
    time.sleep(10)

    mav.stop()
    print("Drone safely disarmed. Launching GUI for data review...")

    # ── Post-flight GUI ───────────────────────────────────────────────
    #app = QtWidgets.QApplication(sys.argv)
    #window = IMUVisualizer(mav)
    #window.show()
    #sys.exit(app.exec_())
