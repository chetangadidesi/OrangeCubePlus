import time
from pymavlink import mavutil

master = mavutil.mavlink_connection('COM8', baud=115200)
master.wait_heartbeat()
print(f"Connected! System: {master.target_system}, Component: {master.target_component}")

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
    msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"ACK: {msg}")

# Test Motor 1 at 20% for 5 seconds
print("Running Motor 1 at 20%...")
actuator_test(4, 0.1, 1)
actuator_test(1,0.1,1)
actuator_test(2, 0.1, 1)
actuator_test(3,0.1,1)
time.sleep(6)
print("Done.")