from pymavlink import mavutil
import time

# Connect to Pixhawk
print("[INFO] Connecting to Pixhawk...")
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()
print(f"[INFO] Connected to system {master.target_system}, component {master.target_component}")

# Set mode to MANUAL
mode_mapping = master.mode_mapping()
if 'MANUAL' not in mode_mapping:
    raise Exception("[ERROR] MANUAL mode not available in mode mapping.")
mode_id = mode_mapping['MANUAL']
master.set_mode(mode_id)
print("[INFO] Set to MANUAL mode.")
time.sleep(1)

# Arm the vehicle
print("[INFO] Arming the vehicle...")
master.arducopter_arm()
master.motors_armed_wait()
print("[SUCCESS] Vehicle armed.")

# RC override array: 8 channels + 10 spares
rc_override = [1500] * 8 + [65535] * 10

# 1. DIVING for 2 seconds
print("[ACTION] Diving for 2 seconds...")
rc_override[2] = 1400  # Vertical thruster down
start_time = time.time()
while time.time() - start_time < 2:
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_override
    )
    time.sleep(0.1)

# Stop diving
rc_override[2] = 1500
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    *rc_override
)
print("[INFO] Diving stopped.")
time.sleep(1)

# 2. MOVING FORWARD for 5 seconds
print("[ACTION] Moving forward for 5 seconds...")
rc_override[4] = 1600  # Forward thrust
rc_override[5] = 1600
start_time = time.time()
while time.time() - start_time < 5:
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_override
    )
    time.sleep(0.1)

# Stop forward motion
rc_override[4] = 1500
rc_override[5] = 1500
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    *rc_override
)
print("[INFO] Forward motion stopped.")
time.sleep(1)

# 3. YAW 720 degrees (8 seconds at 90 deg/sec)
print("[ACTION] Yawing 720 degrees...")
rc_override[3] = 1600  # Yaw thruster
start_time = time.time()
while time.time() - start_time < 8:  # 720 degrees / 90 deg per second = 8 seconds
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_override
    )
    time.sleep(0.1)

# Stop yaw
rc_override[3] = 1500
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    *rc_override
)
print("[INFO] Yaw rotation stopped.")
time.sleep(1)

# 4. SURFACE for 3 seconds
print("[ACTION] Surfacing...")
rc_override[2] = 1600  # Vertical thruster up
start_time = time.time()
while time.time() - start_time < 3:
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_override
    )
    time.sleep(0.1)

# Stop surfacing
rc_override[2] = 1500
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    *rc_override
)
print("[INFO] Surfacing stopped.")
time.sleep(1)

# 5. DISARM the vehicle
print("[INFO] Disarming the vehicle...")
master.arducopter_disarm()
master.motors_disarmed_wait()
print("[SUCCESS] Vehicle disarmed. Mission complete!")