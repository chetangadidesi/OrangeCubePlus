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
    def __init__(self, port='COM8', baud=115200):
        self.port = port
        self.baud = baud
        self.running = False
        self.connection = None
        self.thread = None
        
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
        print("\n\n\n\n")
        
        while self.running:
            # Catch the data transmitted and only get the data of type HIGHRES_IMU and ATTITUDE
            msg = self.connection.recv_match(
                type=['HIGHRES_IMU', 'ATTITUDE'],
                blocking=True,
                timeout=1.0
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
                
                sys.stdout.write("\033[F" * 4) 
                print(f"--- [RAW ACCEL m/s^2] --- X: {self.curr_ax:>6.2f} | Y: {self.curr_ay:>6.2f} | Z: {self.curr_az:>6.2f}    ")
                print(f"--- [RAW GYRO  rad/s] --- X: {self.curr_gx:>6.2f} | Y: {self.curr_gy:>6.2f} | Z: {self.curr_gz:>6.2f}    ")
                print(f"--- [ATTITUDE    deg] --- R: {self.curr_roll:>6.2f} | P: {self.curr_pitch:>6.2f} | Y: {self.curr_yaw:>6.2f}    ")
                print("-" * 60)
                
        
        
        
        


