import sys
import time
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from pymavlink import mavutil

from mav_handler import MAVLinkHandler


class IMUVisualizer(QtWidgets.QMainWindow):
    def __init__(self, mav_handler):
        super().__init__()

        self.mav_handler = mav_handler

        self.setWindowTitle("Cube Orange+ (PX4) - Small Move Test")
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QtWidgets.QVBoxLayout(self.central_widget)

        # Accelerometer
        self.accel_plot = pg.PlotWidget(title="Accelerometer (m/s^2)")
        self.layout.addWidget(self.accel_plot)
        self.accel_plot.addLegend()
        self.accel_plot.setYRange(-15, 15)
        self.curve_ax = self.accel_plot.plot(pen='r', name="X (Front)")
        self.curve_ay = self.accel_plot.plot(pen='g', name="Y (Right)")
        self.curve_az = self.accel_plot.plot(pen='b', name="Z (Down)")

        # Z gyro
        self.gyro_plot = pg.PlotWidget(title="Z-Gyro (rad/s)")
        self.layout.addWidget(self.gyro_plot)
        self.gyro_plot.addLegend()
        self.gyro_plot.setYRange(-10, 10)
        self.curve_gz = self.gyro_plot.plot(pen='y', name="Yaw Rate")

        # Attitude
        self.att_plot = pg.PlotWidget(title="Fused Attitude (Degrees)")
        self.layout.addWidget(self.att_plot)
        self.att_plot.addLegend()
        self.att_plot.setYRange(0, 360)
        self.curve_pitch = self.att_plot.plot(pen=(255, 165, 0), name="Pitch")
        self.curve_roll = self.att_plot.plot(pen=(255, 0, 255), name="Roll")
        self.curve_yaw = self.att_plot.plot(pen=(0, 255, 255), name="Yaw")

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


def disarm_drone(mav):
    print("Disarming...")
    mav.connection.mav.command_long_send(
        mav.connection.target_system,
        mav.connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,  # 0 = disarm
        0, 0, 0, 0, 0, 0
    )


if __name__ == "__main__":

    mav = MAVLinkHandler(port='COM13', baud=57600)
    mav.connect()

    print("Starting telemetry and virtual joystick stream...")
    mav.start()
    time.sleep(1)

    print("Setting Altitude Mode...")
    mav.set_altitude_mode()
    time.sleep(1)

    # Pre-load throttle before arming
    print("Pre-loading throttle to 50%...")
    mav.virtual_roll = 0
    mav.virtual_pitch = 0
    mav.virtual_yaw = 0
    mav.virtual_throttle = 500
    time.sleep(1)

    input("Press ENTER to ARM and start the small movement test...")

    print("Arming...")
    mav.arm()
    time.sleep(0.5)

    try:

        print("\n--- TAKEOFF BLIP ---")
        mav.virtual_roll = 0
        mav.virtual_pitch = 0
        mav.virtual_yaw = 0

        mav.virtual_throttle = 700
        time.sleep(1.0)

        print("--- HOVER BEFORE MOVEMENT ---")
        mav.virtual_throttle = 500
        mav.virtual_pitch = 0
        time.sleep(2.0)

        print("--- SMALL FORWARD MOVEMENT ---")

        mav.virtual_pitch = 150
        mav.virtual_throttle = 500
        time.sleep(0.5)

        print("--- STOPPING / LEVEL HOVER ---")
        mav.virtual_pitch = 0
        mav.virtual_throttle = 500
        time.sleep(2.0)

        print("--- ENDING TEST ---")
        mav.virtual_pitch = 0
        mav.virtual_roll = 0
        mav.virtual_yaw = 0
        mav.virtual_throttle = 0
        time.sleep(0.5)

        disarm_drone(mav)

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt. Stopping immediately.")
        mav.virtual_pitch = 0
        mav.virtual_roll = 0
        mav.virtual_yaw = 0
        mav.virtual_throttle = 0
        time.sleep(0.5)
        disarm_drone(mav)

    # Launch GUI after sequence, same style as main(4).py
    app = QtWidgets.QApplication(sys.argv)
    gui = IMUVisualizer(mav_handler=mav)
    gui.show()

    exit_code = app.exec_()

    print("Shutting down...")
    mav.virtual_throttle = 0
    mav.virtual_pitch = 0
    mav.virtual_roll = 0
    mav.virtual_yaw = 0
    time.sleep(0.5)

    try:
        disarm_drone(mav)
    except Exception as e:
        print(f"Disarm error: {e}")

    sys.exit(exit_code)