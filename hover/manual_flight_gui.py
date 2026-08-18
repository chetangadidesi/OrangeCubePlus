import sys
import time

from PyQt5 import QtWidgets, QtCore, QtGui

from mav_handler import MAVLinkHandler


# ============================================================
# Configuration
# ============================================================


PORT = "COM9"
BAUD = 57600

HOVER_THROTTLE = 500
TAKEOFF_THROTTLE = 750
TAKEOFF_TIME = 1.5

PITCH_COMMAND = 200
ROLL_COMMAND = 400
YAW_COMMAND = 300


# ============================================================
# Hold button
# Sends one command while pressed and returns to zero
# when released.
# ============================================================

class HoldButton(QtWidgets.QPushButton):
    pressed_command = QtCore.pyqtSignal()
    released_command = QtCore.pyqtSignal()

    def __init__(self, text, parent=None):
        super().__init__(text, parent)
        self.setAutoRepeat(False)
        self.setFocusPolicy(QtCore.Qt.NoFocus)

        self.pressed.connect(self.pressed_command.emit)
        self.released.connect(self.released_command.emit)


# ============================================================
# Main GUI
# ============================================================

class ManualFlightGUI(QtWidgets.QMainWindow):

    def __init__(self):
        super().__init__()

        self.mav = None
        self.started = False
        self.stopping = False

        self.setWindowTitle("Cube Orange+ PX4 — Manual Flight Control")
        self.setMinimumSize(850, 650)

        self.build_gui()

    # --------------------------------------------------------
    # GUI
    # --------------------------------------------------------

    def build_gui(self):

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        main_layout = QtWidgets.QVBoxLayout(central)
        main_layout.setSpacing(15)

        # ---------------- STATUS ----------------

        self.status_label = QtWidgets.QLabel("STATUS: STOPPED")
        self.status_label.setAlignment(QtCore.Qt.AlignCenter)
        self.status_label.setStyleSheet(
            "font-size: 20px; font-weight: bold; padding: 10px;"
        )
        main_layout.addWidget(self.status_label)

        # ---------------- START / STOP ----------------

        control_layout = QtWidgets.QHBoxLayout()

        self.start_button = QtWidgets.QPushButton("START")
        self.start_button.setMinimumHeight(60)
        self.start_button.setStyleSheet(
            "font-size: 20px; font-weight: bold;"
        )
        self.start_button.clicked.connect(self.start_flight)

        self.stop_button = QtWidgets.QPushButton("STOP")
        self.stop_button.setMinimumHeight(60)
        self.stop_button.setStyleSheet(
            "font-size: 20px; font-weight: bold;"
        )
        self.stop_button.clicked.connect(self.stop_flight)

        control_layout.addWidget(self.start_button)
        control_layout.addWidget(self.stop_button)

        main_layout.addLayout(control_layout)

        # ---------------- COMMAND DISPLAY ----------------

        self.command_label = QtWidgets.QLabel(
            "Pitch: 0    Roll: 0    Yaw: 0    Throttle: 0"
        )
        self.command_label.setAlignment(QtCore.Qt.AlignCenter)
        self.command_label.setStyleSheet(
            "font-size: 16px; padding: 8px;"
        )
        main_layout.addWidget(self.command_label)

        # ---------------- PADS ----------------

        pads_layout = QtWidgets.QHBoxLayout()
        pads_layout.setSpacing(50)

        # Movement pad
        movement_group = QtWidgets.QGroupBox("MOVEMENT")
        movement_layout = QtWidgets.QGridLayout(movement_group)
        movement_layout.setSpacing(8)

        self.btn_up = HoldButton("↑\nPITCH")
        self.btn_down = HoldButton("↓\nPITCH ")
        self.btn_left = HoldButton("←\nROLL ")
        self.btn_right = HoldButton("→\nROLL ")

        for button in [
            self.btn_up,
            self.btn_down,
            self.btn_left,
            self.btn_right
        ]:
            button.setMinimumSize(150, 100)
            button.setStyleSheet("font-size: 18px;")

        movement_layout.addWidget(self.btn_up, 0, 1)
        movement_layout.addWidget(self.btn_left, 1, 0)
        movement_layout.addWidget(self.btn_right, 1, 2)
        movement_layout.addWidget(self.btn_down, 2, 1)

        # Center indicator
        center = QtWidgets.QLabel("●")
        center.setAlignment(QtCore.Qt.AlignCenter)
        center.setStyleSheet("font-size: 30px;")
        movement_layout.addWidget(center, 1, 1)

        pads_layout.addWidget(movement_group)

        # Yaw pad
        yaw_group = QtWidgets.QGroupBox("YAW")
        yaw_layout = QtWidgets.QHBoxLayout(yaw_group)

        self.btn_yaw_left = HoldButton("←\nYAW ")
        self.btn_yaw_right = HoldButton("→\nYAW ")

        self.btn_yaw_left.setMinimumSize(150, 100)
        self.btn_yaw_right.setMinimumSize(150, 100)

        self.btn_yaw_left.setStyleSheet("font-size: 18px;")
        self.btn_yaw_right.setStyleSheet("font-size: 18px;")

        yaw_layout.addWidget(self.btn_yaw_left)
        yaw_layout.addWidget(self.btn_yaw_right)

        pads_layout.addWidget(yaw_group)

        main_layout.addLayout(pads_layout)

        # ---------------- CONNECTION ----------------

        self.connection_label = QtWidgets.QLabel(
            f"Connection: {PORT} @ {BAUD}"
        )
        self.connection_label.setAlignment(QtCore.Qt.AlignCenter)
        main_layout.addWidget(self.connection_label)

        # ----------------------------------------------------
        # Button signal connections
        # ----------------------------------------------------

        # Movement pad
        self.btn_up.pressed_command.connect(
            lambda: self.set_pitch(PITCH_COMMAND)
        )
        self.btn_up.released_command.connect(
            lambda: self.set_pitch(0)
        )

        self.btn_down.pressed_command.connect(
            lambda: self.set_pitch(-PITCH_COMMAND)
        )
        self.btn_down.released_command.connect(
            lambda: self.set_pitch(0)
        )

        self.btn_right.pressed_command.connect(
            lambda: self.set_roll(ROLL_COMMAND)
        )
        self.btn_right.released_command.connect(
            lambda: self.set_roll(0)
        )

        self.btn_left.pressed_command.connect(
            lambda: self.set_roll(-ROLL_COMMAND)
        )
        self.btn_left.released_command.connect(
            lambda: self.set_roll(0)
        )

        # Yaw pad
        self.btn_yaw_right.pressed_command.connect(
            lambda: self.set_yaw(YAW_COMMAND)
        )
        self.btn_yaw_right.released_command.connect(
            lambda: self.set_yaw(0)
        )

        self.btn_yaw_left.pressed_command.connect(
            lambda: self.set_yaw(-YAW_COMMAND)
        )
        self.btn_yaw_left.released_command.connect(
            lambda: self.set_yaw(0)
        )

        self.set_buttons_enabled(False)

    # --------------------------------------------------------
    # Helpers
    # --------------------------------------------------------

    def set_buttons_enabled(self, enabled):
        buttons = [
            self.btn_up,
            self.btn_down,
            self.btn_left,
            self.btn_right,
            self.btn_yaw_left,
            self.btn_yaw_right
        ]

        for button in buttons:
            button.setEnabled(enabled)

    def update_command_display(self):

        if self.mav is None:
            pitch = roll = yaw = throttle = 0
        else:
            pitch = self.mav.virtual_pitch
            roll = self.mav.virtual_roll
            yaw = self.mav.virtual_yaw
            throttle = self.mav.virtual_throttle

        self.command_label.setText(
            f"Pitch: {pitch:+d}    "
            f"Roll: {roll:+d}    "
            f"Yaw: {yaw:+d}    "
            f"Throttle: {throttle}"
        )

    # --------------------------------------------------------
    # Command setters
    # --------------------------------------------------------

    def set_pitch(self, value):

        if not self.started or self.mav is None:
            return

        self.mav.virtual_pitch = value
        self.update_command_display()

    def set_roll(self, value):

        if not self.started or self.mav is None:
            return

        self.mav.virtual_roll = value
        self.update_command_display()

    def set_yaw(self, value):

        if not self.started or self.mav is None:
            return

        self.mav.virtual_yaw = value
        self.update_command_display()

    # --------------------------------------------------------
    # START
    # --------------------------------------------------------

    def start_flight(self):

        if self.started:
            return

        self.stopping = False
        self.status_label.setText("STATUS: CONNECTING...")

        try:
            self.mav = MAVLinkHandler(
                port=PORT,
                baud=BAUD
            )

            self.mav.connect()

            print("Starting telemetry and virtual joystick stream...")
            self.mav.start()
            time.sleep(1)

            print("Setting Altitude Mode...")
            self.mav.set_altitude_mode()
            time.sleep(1)

            # Neutral controls
            self.mav.virtual_pitch = 0
            self.mav.virtual_roll = 0
            self.mav.virtual_yaw = 0

            # Pre-load hover throttle
            print("Pre-loading throttle to 50%...")
            self.mav.virtual_throttle = HOVER_THROTTLE
            time.sleep(1)

            # Arm
            # NOTE: mav.arm() does not return a value (it only prints the
            # COMMAND_ACK result code to the console), so we can't check
            # a boolean here. Watch the console output for
            # "[ACK] Arm Command Result Code: 0" to confirm PX4 accepted it.
            print("Arming...")
            self.mav.arm()

            time.sleep(0.5)

            # Takeoff blip
            print("Takeoff blip...")
            self.mav.virtual_throttle = TAKEOFF_THROTTLE
            time.sleep(TAKEOFF_TIME)

            # Hover
            print("Hovering...")
            self.mav.virtual_throttle = HOVER_THROTTLE

            self.started = True

            self.status_label.setText("STATUS: ARMED / HOVER")
            self.set_buttons_enabled(True)
            self.update_command_display()

            print("Manual control ready.")

        except Exception as e:

            print(f"START ERROR: {e}")

            self.status_label.setText(
                f"STATUS: ERROR — {e}"
            )

            self.cleanup_after_failed_start()

    # --------------------------------------------------------
    # STOP
    # --------------------------------------------------------

    def stop_flight(self):

        if self.mav is None:
            return

        if self.stopping:
            return

        self.stopping = True
        self.set_buttons_enabled(False)

        print("\n--- STOPPING ---")

        try:
            # First remove all directional commands
            self.mav.virtual_pitch = 0
            self.mav.virtual_roll = 0
            self.mav.virtual_yaw = 0
            self.update_command_display()

            # Drop throttle
            self.mav.virtual_throttle = 0
            time.sleep(0.5)

            # Disarm
            print("Disarming...")
            self.mav.disarm(force=True)
            time.sleep(0.5)

        except Exception as e:
            print(f"STOP ERROR: {e}")

        finally:

            try:
                self.mav.stop()
            except Exception:
                pass

            self.started = False
            self.mav = None
            self.stopping = False

            self.status_label.setText("STATUS: STOPPED")
            self.update_command_display()

            print("Drone stopped.")

    # --------------------------------------------------------
    # Failed START cleanup
    # --------------------------------------------------------

    def cleanup_after_failed_start(self):

        self.set_buttons_enabled(False)

        if self.mav is not None:

            try:
                self.mav.virtual_pitch = 0
                self.mav.virtual_roll = 0
                self.mav.virtual_yaw = 0
                self.mav.virtual_throttle = 0
            except Exception:
                pass

            try:
                self.mav.disarm(force=True)
            except Exception:
                pass

            try:
                self.mav.stop()
            except Exception:
                pass

        self.started = False
        self.mav = None
        self.update_command_display()

    # --------------------------------------------------------
    # Window close
    # --------------------------------------------------------

    def closeEvent(self, event):

        if self.mav is not None:

            try:
                self.mav.virtual_pitch = 0
                self.mav.virtual_roll = 0
                self.mav.virtual_yaw = 0
                self.mav.virtual_throttle = 0
                time.sleep(0.2)
                self.mav.disarm(force=True)
                self.mav.stop()
            except Exception as e:
                print(f"Shutdown error: {e}")

        event.accept()


# ============================================================
# Main
# ============================================================

if __name__ == "__main__":

    app = QtWidgets.QApplication(sys.argv)

    window = ManualFlightGUI()
    window.show()

    sys.exit(app.exec_())