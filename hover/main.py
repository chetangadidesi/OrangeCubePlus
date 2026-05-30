import sys
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import time

# Import the modularized handler
from mav_handler import MAVLinkHandler
from motors import MotorController
from controller import ProportionalController
from pymavlink import mavutil

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
    import sys
    import time
    from PyQt5 import QtWidgets

    mav = MAVLinkHandler(port='COM13', baud=57600)
    mav.connect()

    # 1. Start telemetry and virtual joystick
    print("Starting telemetry and virtual joystick stream...")
    mav.start()
    time.sleep(1)

    # 2. Set to Altitude Mode
    mav.set_altitude_mode()
    time.sleep(1)

    # 3. PRE-LOAD THE THROTTLE TO 50% (HOVER) BEFORE ARMING
    print("Pre-loading throttle to 50% to prevent auto-land disarm...")
    mav.virtual_throttle = 500 
    time.sleep(1) # Let the FC register the mid-stick command

    # 4. ARM THE DRONE
    mav.arm()
    time.sleep(0.5)

    # ==========================================
    # --- AUTOMATED FLIGHT SEQUENCE ---
    # ==========================================
    
    print("\n--- TAKEOFF BLIP (Tricking the Land Detector) ---")
    # Spike throttle to 75% to force the "In Air" state
    mav.virtual_throttle = 750 
    time.sleep(1.5) 
    
    print("--- INITIATING HOVER ---")
    # Drop to 50% to lock in the hover
    mav.virtual_throttle = 500 
    time.sleep(2) 
    
    print("--- FLYING FORWARD ---")
    # Now that it thinks it is flying, command the pitch!
    mav.virtual_pitch = 500  
    time.sleep(3) 
    
    print("--- STOPPING (LEVEL HOVER) ---")
    mav.virtual_pitch = 0    
    # ==========================================
    # Launch GUI while it is "hovering"
    app = QtWidgets.QApplication(sys.argv)
    gui = IMUVisualizer(mav_handler=mav)
    gui.show()
    
    exit_code = app.exec_()
    
    # Clean shutdown
    print("Shutting down... dropping throttle and disarming.")
    mav.virtual_throttle = 0
    time.sleep(0.5)
    
    # Send disarm command (Param 1 = 0)
    mav.connection.mav.command_long_send(
        mav.connection.target_system,
        mav.connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    
    sys.exit(exit_code)
