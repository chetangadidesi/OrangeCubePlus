import sys
import threading
import collections
import collections.abc
import time

collections.MutableMapping = collections.abc.MutableMapping

from pymavlink import mavutil

# Mavlink message IDs for PX4
# https://mavlink.io/en/messages/common.html find more here
HIGHRES_IMU_ID = 105
ATTITUDE_ID = 30
GPS_RAW_INT_ID = 24
GLOBAL_POSITION_INT_ID = 33

class MAVLinkHandler:
    def __init__(self, port='COM13', baud=57600):
        self.port = port
        self.baud = baud
        self.running = False
        self.connection = None
        self.thread = None

        # --- Virtual joystick state (used in manual/altitude modes) ---
        self.virtual_throttle = 0
        self.virtual_pitch = 0
        self.virtual_roll = 0
        self.virtual_yaw = 0

        # --- Offboard state ---
        # When True, _update_data sends position setpoints instead of manual control
        self.offboard_active = False
        self.offboard_lat = 0.0      # degrees
        self.offboard_lon = 0.0      # degrees
        self.offboard_alt = 0.0      # metres (relative)

        # --- Telemetry buffers (sliding window of last 200 samples) ---
        self.data_ax = collections.deque(maxlen=200)
        self.data_ay = collections.deque(maxlen=200)
        self.data_az = collections.deque(maxlen=200)
        self.data_gz = collections.deque(maxlen=200)
        self.data_pitch = collections.deque(maxlen=200)
        self.data_roll  = collections.deque(maxlen=200)
        self.data_yaw   = collections.deque(maxlen=200)

        # --- Latest scalar readings ---
        self.curr_ax, self.curr_ay, self.curr_az = 0.0, 0.0, 0.0
        self.curr_gx, self.curr_gy, self.curr_gz = 0.0, 0.0, 0.0
        self.curr_pitch, self.curr_roll, self.curr_yaw = 0.0, 0.0, 0.0
        
        self.global_lat = None
        self.global_lon = None
        self.global_alt = None
        self.gps_fix_type = 0
        self.gps_sats = 0
        # --- Add this for thread-safe ACKs ---
        self.last_ack = None
        self.fc_custom_mode = 0
        

    # ------------------------------------------------------------------ #
    #  Connection & lifecycle                                              #
    # ------------------------------------------------------------------ #

    def connect(self):
        try:
            self.connection = mavutil.mavlink_connection(self.port, baud=self.baud)
            print("Waiting for heartbeat...")
            self.connection.wait_heartbeat()
            print(f"Connected to system {self.connection.target_system},"
                  f" component {self.connection.target_component}")

            # Core telemetry streams
            self.request_message_interval(HIGHRES_IMU_ID, 100000)       # 10 Hz
            self.request_message_interval(ATTITUDE_ID, 100000)          # 10 Hz
            self.request_message_interval(36, 100000)                   # RC_CHANNELS 10 Hz

            # GPS streams needed for offboard mode
            self.request_message_interval(GPS_RAW_INT_ID, 200000)       # 5 Hz
            self.request_message_interval(GLOBAL_POSITION_INT_ID, 100000)  # 10 Hz
            # Tell the FC to stream position data at 10 Hz
            self.connection.mav.request_data_stream_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                10, # Request 10 Hz
                1   # 1 = Start sending, 0 = Stop sending
            )

            print("All message streams requested.")

        except Exception as e:
            print(f"Connection error: {e}")
            sys.exit()

    def start(self):
        """Start the background telemetry / command thread."""
        self.running = True
        self.thread = threading.Thread(target=self._update_data, daemon=True)
        self.thread.start()

    def stop(self):
        """Stop the background thread."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)

    # ------------------------------------------------------------------ #
    #  MAVLink helpers                                                     #
    # ------------------------------------------------------------------ #

    def request_message_interval(self, message_id, interval_us):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            interval_us,
            0, 0, 0, 0, 0
        )
        print(f"Requested message ID {message_id} at {1e6 / interval_us:.0f} Hz")

    # ------------------------------------------------------------------ #
    #  Background loop                                                     #
    # ------------------------------------------------------------------ #

    def _update_data(self):
        while self.running:
            # 1. GCS heartbeat (keeps FC happy)
            self.connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )

            # 2. Command stream
            # ALWAYS send manual control to keep the FC's RC Loss failsafe happy
            self.connection.mav.manual_control_send(
                self.connection.target_system,
                self.virtual_pitch,
                self.virtual_roll,
                self.virtual_throttle,
                self.virtual_yaw,
                0   # buttons
            )

            # If active, stream position setpoints concurrently
            if self.offboard_active:
                self._send_position_setpoint(
                    self.offboard_lat,
                    self.offboard_lon,
                    self.offboard_alt
                )

            # 3. Receive incoming telemetry
            msg = self.connection.recv_match(
                type=[
                    'HIGHRES_IMU', 'ATTITUDE',
                    'STATUSTEXT',
                    'SERVO_OUTPUT_RAW',
                    'GPS_RAW_INT', 'GLOBAL_POSITION_INT',
                    'HEARTBEAT',
                ],
                blocking=True,
                timeout=0.05
            )

            if not msg:
                continue

            msg_type = msg.get_type()

            if msg_type == 'HIGHRES_IMU':
                self.data_ax.append(msg.xacc)
                self.data_ay.append(msg.yacc)
                self.data_az.append(msg.zacc)
                self.data_gz.append(msg.zgyro)
                self.curr_ax = msg.xacc
                self.curr_ay = msg.yacc
                self.curr_az = msg.zacc
                self.curr_gx = msg.xgyro
                self.curr_gy = msg.ygyro
                self.curr_gz = msg.zgyro

            elif msg_type == 'ATTITUDE':
                self.data_pitch.append(msg.pitch * 57.2958)
                self.data_roll.append(msg.roll * 57.2958)
                yaw_deg = msg.yaw * 57.2958
                if yaw_deg < 0:
                    yaw_deg += 360
                self.data_yaw.append(yaw_deg)
                self.curr_roll  = msg.roll  * 57.2958
                self.curr_pitch = msg.pitch * 57.2958
                self.curr_yaw   = msg.yaw   * 57.2958

            elif msg_type == 'SERVO_OUTPUT_RAW':
                m1, m2 = msg.servo1_raw, msg.servo2_raw
                m3, m4 = msg.servo3_raw, msg.servo4_raw
                sys.stdout.write(f"\r[MOTORS] M1:{m1} | M2:{m2} | M3:{m3} | M4:{m4}    ")
                sys.stdout.flush()
                
            elif msg_type == 'GPS_RAW_INT':
                self.gps_fix_type = msg.fix_type
                self.gps_sats = msg.satellites_visible

            elif msg_type == 'GLOBAL_POSITION_INT':
                self.global_lat = msg.lat / 1e7
                self.global_lon = msg.lon / 1e7
                self.global_alt = msg.relative_alt / 1000.0
                
            elif msg_type == 'COMMAND_ACK':
                self.last_ack = msg.result

            elif msg_type == 'STATUSTEXT':
                print(f"\n[FC ALERT]: {msg.text}")
            
            elif msg_type == 'HEARTBEAT':
                # Ignore our own GCS heartbeat, only read the drone's
                if msg.type != mavutil.mavlink.MAV_TYPE_GCS:
                    self.fc_custom_mode = msg.custom_mode

    # ------------------------------------------------------------------ #
    #  Arming                                                              #
    # ------------------------------------------------------------------ #

    def arm(self):
        """Force-arm the drone and verify via Heartbeat."""
        print("Sending FORCE Arm command...")
        
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,      # Arm
            21196,  # Force bypass
            0, 0, 0, 0, 0
        )

        print("Waiting for Arm confirmation via Heartbeat...")
        
        start_time = time.time()
        while time.time() - start_time < 5:
            # pymavlink automatically tracks the armed state from incoming heartbeats
            if self.connection.motors_armed():
                print("Success! FC confirmed ARMED state.")
                return True
            time.sleep(0.1)
            
        print("Arm confirmation timeout. FC is still DISARMED.")
        return False

    def disarm(self, force=True):
        """Disarm the drone. force=True bypasses safety checks."""
        print("Sending Disarm command...")
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,                  # Disarm
            21196 if force else 0,
            0, 0, 0, 0, 0
        )

    # ------------------------------------------------------------------ #
    #  Flight modes                                                        #
    # ------------------------------------------------------------------ #

    def set_stabilized_mode(self):
        """Stabilized mode — no GPS needed; pilot manages altitude."""
        print("Requesting Stabilized Mode...")
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            1,   # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            7,   # PX4_CUSTOM_MAIN_MODE_STABILIZED
            0, 0, 0, 0, 0
        )

    def set_altitude_mode(self):
        """Altitude-hold mode — baro-based height hold, pilot manages position."""
        print("Requesting Altitude Mode...")
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            1,   # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            2,   # PX4_CUSTOM_MAIN_MODE_ALTCTL
            0, 0, 0, 0, 0
        )

    def set_offboard_mode(self):
        """
        Switch PX4 into OFFBOARD mode using raw constants.
        """
        print("Requesting OFFBOARD mode via raw MAVLink constants...")
        
        PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
        
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            1, # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            PX4_CUSTOM_MAIN_MODE_OFFBOARD,
            0, 0, 0, 0, 0
        )

        print("Waiting for mode confirmation via raw Heartbeat...")
        
        start_time = time.time()
        while time.time() - start_time < 5:
            # PX4 packs the main mode in the 3rd byte of custom_mode.
            # We bit-shift by 16 to read it.
            main_mode = (self.fc_custom_mode >> 16) & 0xFF
            
            if main_mode == PX4_CUSTOM_MAIN_MODE_OFFBOARD:
                print("Success! FC confirmed OFFBOARD mode.")
                return True
                
            time.sleep(0.1)

        print(f"Mode switch timeout. FC raw custom_mode is stuck at: {self.fc_custom_mode}")
        return False

    # ------------------------------------------------------------------ #
    #  GPS / position helpers                                              #
    # ------------------------------------------------------------------ #

    # ------------------------------------------------------------------ #
    #  GPS / position helpers                                            #
    # ------------------------------------------------------------------ #

    def wait_for_gps_fix(self, min_fix_type=3, timeout_sec=30):
        """
        Block until background thread reports at least `min_fix_type` (3 = 3-D fix).
        """
        print(f"Waiting for GPS fix (min fix type = {min_fix_type})...")
        start_time = time.time()
        
        while True:
            if time.time() - start_time > timeout_sec:
                print("Timeout: Could not acquire GPS fix.")
                return False
                
            # Check the variables updated by _update_data
            if self.gps_fix_type >= min_fix_type:
                print(f"GPS fix acquired! Fix: {self.gps_fix_type}, Sats: {self.gps_sats}")
                return True
                
            time.sleep(0.5)

    def read_global_position(self, timeout_sec=150):
        """
        Reads the current fused GLOBAL_POSITION_INT from the background thread variables.
        Returns (lat_deg, lon_deg, alt_m) or None if it times out.
        """
        print("Waiting for EKF to publish GLOBAL_POSITION_INT...")
        timeout_start = time.time()
        
        while True:
            if time.time() - timeout_start > timeout_sec:
                print("Timeout: Could not read GLOBAL_POSITION_INT.")
                return None
                
            # If the background thread has populated these, the EKF is ready!
            if self.global_lat is not None and self.global_alt is not None:
                return self.global_lat, self.global_lon, self.global_alt
                
            time.sleep(0.2)
    # ------------------------------------------------------------------ #
    #  Offboard position control                                           #
    # ------------------------------------------------------------------ #

    def _send_position_setpoint(self, lat, lon, rel_alt):
        """
        Send SET_POSITION_TARGET_GLOBAL_INT — the message PX4 needs to
        keep OFFBOARD mode alive.  Called automatically by _update_data
        when self.offboard_active is True.
        """
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        self.connection.mav.set_position_target_global_int_send(
            int(time.time() * 1000) & 0xFFFFFFFF,   # time_boot_ms
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            int(lat * 1e7),   # lat_int
            int(lon * 1e7),   # lon_int
            rel_alt,          # alt  (relative metres)
            0, 0, 0,          # vx vy vz (ignored)
            0, 0, 0,          # afx afy afz (ignored)
            0, 0              # yaw yaw_rate (ignored)
        )

    def set_offboard_setpoint(self, lat, lon, rel_alt):
        """
        Update the position target that _update_data will keep streaming.
        Safe to call at any time; takes effect on the next loop iteration.
        """
        self.offboard_lat = lat
        self.offboard_lon = lon
        self.offboard_alt = rel_alt

    # Add takeoff_alt to the arguments (default is 0 for ground tests)
    def init_offboard(self, pre_stream_sec=3, takeoff_alt=0.0):
        """
        Full offboard initialisation sequence utilizing the background thread.
        """
        # Step 1 — GPS fix
        self.wait_for_gps_fix(min_fix_type=3)

        # Step 2 — Read fused position
        pos = self.read_global_position()
        if pos is None:
            raise RuntimeError("Could not read GLOBAL_POSITION_INT.")

        lat, lon, rel_alt = pos
        
        # --- NEW: Calculate the target altitude in the air ---
        target_alt = rel_alt + takeoff_alt 
        
        print(f"\nOffboard target setpoint → Lat: {lat:.7f}, Lon: {lon:.7f}, RelAlt: {target_alt:.2f} m")
        
        # Set the background thread to stream the IN-AIR target
        self.set_offboard_setpoint(lat, lon, target_alt)

        # Step 3 — Hand off to background thread for pre-streaming
        self.offboard_active = True
        print(f"Pre-streaming setpoints in background for {pre_stream_sec} s...")
        time.sleep(pre_stream_sec)

        # Step 4 — request mode switch
        print("Requesting OFFBOARD mode...")
        ok = self.set_offboard_mode()
        
        if not ok:
            print("OFFBOARD mode rejected by FC.")
            self.offboard_active = False # Turn it off to fall back safely
            return False

        print("OFFBOARD mode active. Background thread is maintaining hold.")
        return True

    def exit_offboard(self):
        """
        Disable the offboard setpoint stream and switch back to Altitude mode.
        Call this before landing or handing back manual control.
        """
        print("Exiting OFFBOARD mode...")
        self.offboard_active = False
        time.sleep(0.1)            # let the thread finish the current iteration
        self.set_altitude_mode()   # hand control back to a safe mode
        print("Switched back to Altitude mode.")

    # ------------------------------------------------------------------ #
    #  Diagnostics                                                         #
    # ------------------------------------------------------------------ #

    def print_heartbeat_state(self, label="State"):
        """Print armed status and flight mode from the next HEARTBEAT."""
        msg = self.connection.recv_match(
            type="HEARTBEAT", blocking=True, timeout=3
        )
        if msg is None:
            print(f"{label}: No heartbeat received.")
            return
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f"{label}:")
        print(f"  base_mode:     {msg.base_mode}")
        print(f"  custom_mode:   {msg.custom_mode}")
        print(f"  armed:         {armed}")
        print(f"  system_status: {msg.system_status}")

    def print_status_messages(self, seconds=3):
        """Drain and print STATUSTEXT messages for `seconds` seconds."""
        print(f"Reading FC status messages for {seconds} s...")
        start = time.time()
        while time.time() - start < seconds:
            msg = self.connection.recv_match(
                type="STATUSTEXT", blocking=True, timeout=1
            )
            if msg is not None:
                print(f"[FC STATUS] {msg.text}")
