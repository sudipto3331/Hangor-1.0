# Importing Libraries
from pymavlink import mavutil
import time

# Connect to Pixhawk
print("[INFO] Connect to Pixhawk")
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()
print(f"[INFO] Connected to system {master.target_system}, component {master.target_component}")

# Arm Vehicle
print("[INFO] Arming the vehicle")
master.arducopter_arm()
master.motors_armed_wait()
print("[INFO] Vehicle armed successfully")

# RC override array: 8 channels + 10 spares
rc_override = [1500]*8 + [65535]*10

# Applying forward thrust
rc_override[4] = 1600
rc_override[5] = 1600

print("[ACTION] Moving forward for 5 seconds...")
start_time = time.time()
while time.time() - start_time < 5:
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_override
    )
    time.sleep(0.1)

# Stop motion
rc_override[4] = 1500
rc_override[5] = 1500
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    *rc_override
)
print("[INFO] Motion stopped.")

# Disarm the vehicle
print("[INFO] Disarming the vehicle.")
master.arducopter_disarm()
master.motors_disarmed_wait()
print("[SUCCESS] Vehicle disarmed.")
