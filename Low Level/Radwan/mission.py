from pymavlink import mavutil
import time

# Connect to Pixhawk
print("[INFO] Connecting to Pixhawk...")
master = mavutil.mavlink_connection('udpin:192.168.2.2:14550')
master.wait_heartbeat()
print(f"[INFO] Connected to system {master.target_system}, component {master.target_component}")

# Set to MANUAL mode
mode_mapping = master.mode_mapping()
if 'MANUAL' not in mode_mapping:
    raise Exception("[ERROR] MANUAL mode not available.")
mode_id = mode_mapping['MANUAL']
master.set_mode(mode_id)
print("[INFO] Mode set to MANUAL.")
time.sleep(1)

# Arm the vehicle
print("[INFO] Arming the vehicle...")
master.arducopter_arm()
master.motors_armed_wait()
print("[SUCCESS] Vehicle armed.")

# RC override: 8 main + 10 unused
rc_override = [1500] * 8 + [65535] * 10

# DIVE for 2 seconds (Throttle down)
rc_override[2] = 1600  # Dive (increase throttle)
print("[ACTION] Diving for 2 seconds...")
start = time.time()
while time.time() - start < 2:
    master.mav.rc_channels_override_send(master.target_system, master.target_component, *rc_override)
    time.sleep(0.1)

# Forward motion for 5 seconds
rc_override[2] = 1500  # Neutral throttle
rc_override[4] = 1600  # Forward thrust
rc_override[5] = 1600
print("[ACTION] Moving forward for 5 seconds...")
start = time.time()
while time.time() - start < 5:
    master.mav.rc_channels_override_send(master.target_system, master.target_component, *rc_override)
    time.sleep(0.1)

# Stabilize for 3 seconds
rc_override[4] = 1500
rc_override[5] = 1500
print("[ACTION] Stabilizing for 3 seconds...")
start = time.time()
while time.time() - start < 3:
    master.mav.rc_channels_override_send(master.target_system, master.target_component, *rc_override)
    time.sleep(0.1)

# Yaw 720 degrees (slow yaw for ~6 seconds)
rc_override[3] = 1700  # Yaw right
print("[ACTION] Yawing 720° (approx 6 seconds)...")
start = time.time()
while time.time() - start < 6:
    master.mav.rc_channels_override_send(master.target_system, master.target_component, *rc_override)
    time.sleep(0.1)

# Stabilize 2s
rc_override[3] = 1500
print("[ACTION] Stabilizing for 2 seconds...")
start = time.time()
while time.time() - start < 2:
    master.mav.rc_channels_override_send(master.target_system, master.target_component, *rc_override)
    time.sleep(0.1)

# Surface (Throttle up)
rc_override[2] = 1400  # Come back up
print("[ACTION] Surfacing for 2 seconds...")
start = time.time()
while time.time() - start < 2:
    master.mav.rc_channels_override_send(master.target_system, master.target_component, *rc_override)
    time.sleep(0.1)

# Stop all motion
rc_override = [1500] * 8 + [65535] * 10
master.mav.rc_channels_override_send(master.target_system, master.target_component, *rc_override)
print("[INFO] Motion stopped.")

# Disarm
print("[INFO] Disarming vehicle...")
master.arducopter_disarm()
master.motors_disarmed_wait()
print("[SUCCESS] Vehicle disarmed.")
