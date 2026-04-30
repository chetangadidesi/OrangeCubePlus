import sys
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

# Import the modularized handler
from mavlink_handler import MAVLinkHandler

class IMUVisualizer(QtWidgets.QMainWindow):
    def __init__(self, mav_handler):
        super().__init__()
        
        # Store the reference to our communication handler
        self.mav_handler = mav_handler

        self.setWindowTitle("Cube Orange+ (PX4)")
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QtWidgets.QVBoxLayout(self.central_widget)

        # --- PLOT 1: ACCELEROMETER ---
        self.accel_plot = pg.PlotWidget(title="Accelerometer (m/s^2)")
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

        # --- PLOT 3: ATTITUDE (Angles) ---
        self.att_plot = pg.PlotWidget(title="Fused Attitude (Degrees)")
        self.layout.addWidget(self.att_plot)
        self.att_plot.addLegend()
        self.att_plot.setYRange(0, 360)
        self.curve_pitch = self.att_plot.plot(pen=(255, 165, 0), name="Pitch")
        self.curve_roll  = self.att_plot.plot(pen=(255, 0, 255),  name="Roll")
        self.curve_yaw   = self.att_plot.plot(pen=(0, 255, 255),  name="Yaw")

        # --- Timer for UI Updates ---
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(30)

    def update_plot(self):
        # Pull data from the MAVLinkHandler's buffers
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
        # Stop the background thread before the GUI closes
        self.mav_handler.stop()
        super().closeEvent(event)


if __name__ == "__main__":
    # 1. Initialize and start MAVLink Communication
    mav = MAVLinkHandler(port='COM8', baud=115200)
    mav.connect()
    mav.start()

    # 2. Initialize GUI and inject the MAVLink handler
    app = QtWidgets.QApplication(sys.argv)
    gui = IMUVisualizer(mav_handler=mav)
    gui.show()
    
    # 3. Execute application loop
    sys.exit(app.exec_())