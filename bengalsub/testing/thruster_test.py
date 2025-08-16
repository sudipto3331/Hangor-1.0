from pymavlink import mavutil
import time

# Connect to Pixhawk
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()
print(f"Connected to system {master.target_system}")

# Set to MANUAL mode and arm
mode = 'MANUAL'
mode_id = master.mode_mapping()[mode]
master.set_mode(mode_id)
print("Set to MANUAL mode")

master.arducopter_arm()
master.motors_armed_wait()
print("Vehicle armed")

# Initialize all thrusters to neutral (1500 = stop)
thrusters = [1500] * 8 + [65535] * 10  # 8 thrusters + 10 unused channels

def set_thruster(thruster_num, pwm_value):
    """Set individual thruster PWM (thruster_num: 1-8, pwm_value: 1100-1900)"""
    thrusters[thruster_num - 1] = pwm_value
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component, *thrusters
    )
    print(f"Thruster {thruster_num}: {pwm_value} PWM")

def stop_all():
    """Stop all thrusters"""
    for i in range(8):
        thrusters[i] = 1500
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component, *thrusters
    )
    print("All thrusters stopped")

# Example usage:
try:
    # Control individual thrusters
    set_thruster(1, 1600)  # Thruster 1 forward
    time.sleep(2)
    
    # set_thruster(2, 1400)  # Thruster 2 reverse  
    # time.sleep(2)
    
    # set_thruster(3, 1700)  # Thruster 3 fast forward
    # time.sleep(2)
    
    # Stop all thrusters
    stop_all()
    
except KeyboardInterrupt:
    print("Emergency stop!")
    stop_all()

finally:
    # Always stop and disarm
    stop_all()
    time.sleep(1)
    master.arducopter_disarm()
    print("Disarmed and stopped")
    master.close()