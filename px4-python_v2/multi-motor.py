import time
from pymavlink import mavutil

master = mavutil.mavlink_connection('COM13', baud=57600)
master2 = mavutil.mavlink_connection('COM14', baud=57600)
master.wait_heartbeat()
master2.wait_heartbeat()
print(f"Connected! System: {master.target_system}, Component: {master.target_component}")
print(f"Connected! System: {master2.target_system}, Component: {master2.target_component}")
def actuator_test( motor_number, value, duration):
    """
    motor_index : 1-based (Motor 1 = 1)
    value       : 0.0 to 1.0 (0 = off, 1 = full throttle)
    duration    : seconds (0 = needs to be sent repeatedly)
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        310,  # MAV_CMD_ACTUATOR_TEST
        0,
        value,        # param1: output value (0.0 - 1.0)
        duration,     # param2: timeout in seconds
        0, 0,
        motor_number,            # param5: actuator type (1 = motor)
        0,  # param6: motor index (1-based)
        0
    )
    
def actuator_test2( motor_number, value, duration):
    """
    motor_index : 1-based (Motor 1 = 1)
    value       : 0.0 to 1.0 (0 = off, 1 = full throttle)
    duration    : seconds (0 = needs to be sent repeatedly)
    """
    master2.mav.command_long_send(
        master2.target_system,
        master2.target_component,
        310,  # MAV_CMD_ACTUATOR_TEST
        0,
        value,        # param1: output value (0.0 - 1.0)
        duration,     # param2: timeout in seconds
        0, 0,
        motor_number,            # param5: actuator type (1 = motor)
        0,  # param6: motor index (1-based)
        0
    )

def get_gps():
    # Primary GPS (GPS1)
    gps1 = master2.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
    
    # Secondary GPS (GPS2)
    gps2 = master2.recv_match(type='GPS2_RAW', blocking=True, timeout=5)
    
    fix_types = {0: "No fix", 1: "No fix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK float", 6: "RTK fixed"}
    
    if gps1:
        print(f"GPS1 → Fix: {fix_types.get(gps1.fix_type)}, "
              f"Sats: {gps1.satellites_visible}, "
              f"Lat: {gps1.lat/1e7}, Lon: {gps1.lon/1e7}")
    
    if gps2:
        print(f"GPS2 → Fix: {fix_types.get(gps2.fix_type)}, "
              f"Sats: {gps2.satellites_visible}, "
              f"Lat: {gps2.lat/1e7}, Lon: {gps2.lon/1e7}")

    # msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    # print(f"ACK: {msg}")


print("Running Motor 1 at 20%...")
actuator_test(4, 0.1, 1)
actuator_test(1,0.1,1)
actuator_test(2, 0.1, 1)
actuator_test(3,0.1,1)


actuator_test2(4, 0.1, 1)
actuator_test2(1,0.1,1) 
actuator_test2(2, 0.1, 1)
actuator_test2(3,0.1,1)
get_gps()
#time.sleep(6)
print("Done.")
