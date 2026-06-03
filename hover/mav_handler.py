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

class MAVLinkHandler:
    def __init__(self, port='COM13', baud=57600):
        self.port = port
        self.baud = baud
        self.running = False
        self.connection = None
        self.thread = None
        self.virtual_throttle = 0
        self.virtual_pitch = 0
        self.virtual_roll = 0
        self.virtual_yaw = 0
        # Sliding window to only store last 200 data elements from the continuous stream
        self.data_ax = collections.deque(maxlen=200) # Acceleration in x
        self.data_ay = collections.deque(maxlen=200) # Acceleration in y
        self.data_az = collections.deque(maxlen=200) # Acceleration in z
        self.data_gz = collections.deque(maxlen=200) # Rotation along z 
        self.data_pitch = collections.deque(maxlen=200) # Pitch 
        self.data_roll = collections.deque(maxlen=200) # Roll
        self.data_yaw = collections.deque(maxlen=200)   # Yaw
        self.curr_ax, self.curr_ay, self.curr_az = 0.0, 0.0, 0.0
        self.curr_gx, self.curr_gy, self.curr_gz = 0.0, 0.0, 0.0
        self.curr_pitch, self.curr_roll, self.curr_yaw = 0.0, 0.0, 0.0
    
    def connect(self):
        try:
            # Connecting to OrangeCube+ connected to port COM8 with USB baudrate 
            self.connection = mavutil.mavlink_connection(self.port, baud= self.baud)
            print("Waiting for heartbeat")
            self.connection.wait_heartbeat()
            print("Connected to system")  # Connected to OrangeCube +
            # Request data from Orange Cube + 
            self.request_message_interval(HIGHRES_IMU_ID, 100000) # requesting IMU data 10Hz
            self.request_message_interval(ATTITUDE_ID, 100000) # requesting Attitude data 10Hz
            print("Requested message ID 36 at 10 Hz")
            self.request_message_interval(36, 100000)
            print("Data requested")
        
        except Exception as e:
            print(f"Connection error: {e}")
            sys.exit()
            
    def start(self):
        # Starting the background thread for fetching data
        self.running = True
        self.thread = threading.Thread(target=self._update_data, daemon=True)
        self.thread.start()
    
    def stop(self):
        # Stops the background thread
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
            
    def request_message_interval(self, message_id, interval_us):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            interval_us,
            0,0,0,0,0
        )
        print(f"Requested message ID {message_id} at {1e6/interval_us: .0f} Hz")
        
    def _update_data(self):
        while self.running:
            # 1. SEND GCS HEARTBEAT
            self.connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )

            # 2. BROADCAST VIRTUAL JOYSTICK STATE
            self.connection.mav.manual_control_send(
                self.connection.target_system,
                self.virtual_pitch,  # Pitch 
                self.virtual_roll,  # Roll 
                self.virtual_throttle, # Our dynamic throttle variable
                self.virtual_yaw,  # Yaw 
                0   # Buttons 
            )

            # 3. CATCH INCOMING DATA
            msg = self.connection.recv_match(
                type=['HIGHRES_IMU', 'ATTITUDE', 'COMMAND_ACK', 'STATUSTEXT', 'SERVO_OUTPUT_RAW'],
                blocking=True,
                timeout=0.05
            )
            if msg:
                msg_type = msg.get_type() # Get the type of the data
                
                # if HIGHRES_IMU then we assign its components to each variable
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

                    
                # Similarly for ATTITUDE
                elif msg_type == 'ATTITUDE':
                    self.data_pitch.append(msg.pitch * 57.2958)
                    self.data_roll.append(msg.roll * 57.2958)
                    yaw_deg = msg.yaw * 57.2958
                    if yaw_deg < 0:
                        yaw_deg +=360
                    self.data_yaw.append(yaw_deg)   
                    self.curr_roll = msg.roll * 57.2958
                    self.curr_pitch = msg.pitch * 57.2958
                    self.curr_yaw = msg.yaw * 57.2958
                
                elif msg_type == 'SERVO_OUTPUT_RAW':
                    # Extract the first 4 motor PWM values
                    m1 = msg.servo1_raw
                    m2 = msg.servo2_raw
                    m3 = msg.servo3_raw
                    m4 = msg.servo4_raw
                    
                    # Printing servo output
                    sys.stdout.write(f"\r[MOTORS] M1:{m1} | M2:{m2} | M3:{m3} | M4:{m4}    ")
                    sys.stdout.flush()
                
                elif msg_type == 'STATUSTEXT':
                    # This will print any errors that happen DURING the flight
                    print(f"\n[FC ALERT]: {msg.text}")
                    
    def arm(self):
        """
        FORCE ARMS the drone, bypassing all safety checks.
        PROPELLERS MUST BE OFF.
        """
        print("Sending FORCE Arm command...")
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, # Confirmation
            1, # Param 1: 1 = Arm
            21196, # Param 2: FORCE ARM NUMBER
            0, 0, 0, 0, 0
        )
        
        
        # Diagnostic loop: listen to EVERYTHING for 2.5 seconds
        start_time = time.time()
        print("Listening for flight controller responses...")
        
        while time.time() - start_time < 2.5:
            # blocking=False allows us to churn through the buffer quickly
            msg = self.connection.recv_match(blocking=False)
            if msg:
                mtype = msg.get_type()
                
                # Catch plain-text errors (like "Preflight Fail: No RC")
                if mtype == 'STATUSTEXT':
                    print(f"[FC TEXT] {msg.text}")
                    
                # Catch formal command acknowledgments
                elif mtype == 'COMMAND_ACK':
                    if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                        print(f"[ACK] Arm Command Result Code: {msg.result}")
                    elif msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                        print(f"[ACK] Mode Change Result Code: {msg.result}")
                    
    def set_stabilized_mode(self):
        """
        Sets the PX4 flight mode to Stabilized.
        Does NOT require a GPS lock. Keeps the drone level, but the user must manage altitude.
        """
        # MAV_CMD_DO_SET_MODE = 176
        # param1 = 1 (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
        # param2 = 7 (PX4_CUSTOM_MAIN_MODE_STABILIZED)
        # param3 = 0 (No sub-mode needed)
        
        print("Requesting Stabilized Mode...")
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0, # Confirmation
            1, # Param 1: Custom mode enabled
            7, # Param 2: Main mode STABILIZED
            0, # Param 3: Sub mode 0
            0, 0, 0, 0 # Params 4-7 (unused)
        )
        
    def set_altitude_mode(self):
        print("Requesting Altitude Mode...")
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0, # Confirmation
            1, # Param 1: Custom mode enabled
            2, # Param 2: Main mode ALTCTL (Altitude)
            0, # Param 3: Sub mode 0
            0, 0, 0, 0
        )
