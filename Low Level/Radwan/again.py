from pymavlink import mavutil
import time

# Connect to Pixhawk
print("[INFO] Connecting to Pixhawk...")
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')  # Use serial for autonomous
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

# 1. Arm the vehicle
print("[1] Arming the vehicle...")
master.arducopter_arm()
master.motors_armed_wait()
print("[SUCCESS] Vehicle armed.")

# RC override array: 8 channels + 10 spares
rc_override = [1500] * 8 + [65535] * 10

# 2. Dive for 2 seconds
print("[2] Diving for 2 seconds...")
rc_override[2] = 1400  # Throttle down
start_time = time.time()
while time.time() - start_time < 2:
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_override
    )
    time.sleep(0.1)

# Stop dive
rc_override[2] = 1500
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    *rc_override
)

# 3. Stabilize for 3 seconds
print("[3] Stabilizing for 3 seconds...")
time.sleep(3)

# 4. Move forward for 2 seconds
print("[4] Moving forward for 2 seconds...")
rc_override[4] = 1600  # Forward thrust
rc_override[5] = 1600
start_time = time.time()
while time.time() - start_time < 2:
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

# 5. Stabilize for 3 seconds
print("[5] Stabilizing for 3 seconds...")
time.sleep(3)

# 6. Yaw right for 2 seconds
print("[6] Yawing right for 2 seconds...")
rc_override[4] = 1600  # One thruster forward
rc_override[5] = 1400  # One thruster reverse
start_time = time.time()
while time.time() - start_time < 2:
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_override
    )
    time.sleep(0.1)

# Stop yaw
rc_override[4] = 1500
rc_override[5] = 1500
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    *rc_override
)

# 7. Come to surface for 2 seconds
print("[7] Coming to surface for 2 seconds...")
rc_override[2] = 1600  # Throttle up
start_time = time.time()
while time.time() - start_time < 2:
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_override
    )
    time.sleep(0.1)

# Stop surface motion
rc_override[2] = 1500
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    *rc_override
)
print("[INFO] All motions stopped.")

# 8. Disarm the vehicle
print("[8] Disarming the vehicle...")
master.arducopter_disarm()
master.motors_disarmed_wait()
print("[SUCCESS] Mission complete. Vehicle disarmed.")