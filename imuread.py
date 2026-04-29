import sys
import collections
import collections.abc

# Fix for Python 3.10+
collections.MutableMapping = collections.abc.MutableMapping

from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from pymavlink import mavutil
import threading

class IMUVisualizer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Cube Orange+ ")
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QtWidgets.QVBoxLayout(self.central_widget)

        # --- PLOT 1: ACCELEROMETER ---
        self.accel_plot = pg.PlotWidget(title="Accelerometer (m/s^2)")
        self.layout.addWidget(self.accel_plot)
        self.accel_plot.addLegend() # Added Legend
        self.accel_plot.setYRange(-15, 15)
        self.data_ax, self.data_ay, self.data_az = [collections.deque(maxlen=200) for _ in range(3)]
        self.curve_ax = self.accel_plot.plot(pen='r', name="X (Front)")
        self.curve_ay = self.accel_plot.plot(pen='g', name="Y (Right)")
        self.curve_az = self.accel_plot.plot(pen='b', name="Z (Down)")

        # --- PLOT 2: Z-GYRO ---
        self.gyro_plot = pg.PlotWidget(title="Z-Gyro (rad/s)")
        self.layout.addWidget(self.gyro_plot)
        self.gyro_plot.addLegend() # Added Legend
        self.gyro_plot.setYRange(-5, 5)
        self.data_gz = collections.deque(maxlen=200)
        self.curve_gz = self.gyro_plot.plot(pen='y', name="Yaw Rate")

        # --- PLOT 3: ATTITUDE (Angles) ---
        self.att_plot = pg.PlotWidget(title="Fused Attitude (Degrees)")
        self.layout.addWidget(self.att_plot)
        self.att_plot.addLegend() # Added Legend
        self.att_plot.setYRange(-180, 180) 
        self.data_pitch, self.data_roll, self.data_yaw = [collections.deque(maxlen=200) for _ in range(3)]
        self.curve_pitch = self.att_plot.plot(pen=(255, 165, 0), name="Pitch")
        self.curve_roll = self.att_plot.plot(pen=(255, 0, 255), name="Roll")
        self.curve_yaw = self.att_plot.plot(pen=(0, 255, 255), name="Yaw") # Cyan

        # --- MAVLink Connection ---
        try:
            self.connection = mavutil.mavlink_connection('COM10', baud=115200)
            print("Connected to Cube!")
        except Exception as e:
            print(f"Error: {e}")
            sys.exit()

        self.running = True
        self.thread = threading.Thread(target=self.update_data, daemon=True)
        self.thread.start()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(30)

    def update_data(self):
        while self.running:
            msg = self.connection.recv_match(type=['RAW_IMU', 'ATTITUDE'], blocking=True, timeout=1.0)
            if msg:
                if msg.get_type() == 'RAW_IMU':
                    self.data_ax.append(msg.xacc / 102.0)
                    self.data_ay.append(msg.yacc / 102.0)
                    self.data_az.append(msg.zacc / 102.0)
                    self.data_gz.append(msg.zgyro / 1000.0)
                
                elif msg.get_type() == 'ATTITUDE':
                    # Pitch and Roll are fine as-is
                    self.data_pitch.append(msg.pitch * 57.2958)
                    self.data_roll.append(msg.roll * 57.2958)
                    
                    # Normalize Yaw to 0-360 so it doesn't jump between negative and positive
                    yaw_deg = msg.yaw * 57.2958
                    if yaw_deg < 0:
                        yaw_deg += 360
                    self.data_yaw.append(yaw_deg)

                # Also, update your Plot 3 Y-Range to show 0 to 360
                self.att_plot.setYRange(0, 360)

    def update_plot(self):
        if len(self.data_ax) > 0:
            self.curve_ax.setData(list(self.data_ax))
            self.curve_ay.setData(list(self.data_ay))
            self.curve_az.setData(list(self.data_az))
            self.curve_gz.setData(list(self.data_gz))
        if len(self.data_pitch) > 0:
            self.curve_pitch.setData(list(self.data_pitch))
            self.curve_roll.setData(list(self.data_roll))
            self.curve_yaw.setData(list(self.data_yaw))

    def closeEvent(self, event):
        self.running = False
        super().closeEvent(event)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    gui = IMUVisualizer()
    gui.show()
    sys.exit(app.exec_())