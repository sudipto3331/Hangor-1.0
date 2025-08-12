# from pymavlink import mavutil
# import time

# # Connect to Pixhawk
# print("[INFO] Connecting to Pixhawk...")
# master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# master.wait_heartbeat()
# print(f"[INFO] Connected to system {master.target_system}, component {master.target_component}")

# # Set to MANUAL mode
# mode_mapping = master.mode_mapping()
# if 'MANUAL' not in mode_mapping:
#     raise Exception("[ERROR] MANUAL mode not available.")
# mode_id = mode_mapping['MANUAL']
# master.set_mode(mode_id)
# print("[INFO] Mode set to MANUAL.")
# time.sleep(1)

# # Arm the vehicle
# print("[INFO] Arming the vehicle...")
# master.arducopter_arm()
# master.motors_armed_wait()
# print("[SUCCESS] Vehicle armed.")

# # RC override: 8 main + 10 unused
# rc_override = [1500] * 8 + [65535] * 10

# # DIVE for 2 seconds (Throttle down)
# rc_override[2] = 1600  # Dive (increase throttle)
# print("[ACTION] Diving for 5 seconds...")
# start = time.time()
# while time.time() - start < 5:
#     master.mav.rc_channels_override_send(master.target_system, master.target_component, *rc_override)
#     time.sleep(1)

# # Forward motion for 5 seconds
# rc_override[2] = 1500  # Neutral throttle
# rc_override[4] = 1600  # Forward thrust
# rc_override[5] = 1600
# print("[ACTION] Moving forward for 5 seconds...")
# start = time.time()
# while time.time() - start < 5:
#     master.mav.rc_channels_override_send(master.target_system, master.target_component, *rc_override)
#     time.sleep(1)

# # Stabilize for 3 seconds
# rc_override[4] = 1500
# rc_override[5] = 1500
# print("[ACTION] Stabilizing for 3 seconds...")
# start = time.time()
# while time.time() - start < 3:
#     master.mav.rc_channels_override_send(master.target_system, master.target_component, *rc_override)
#     time.sleep(1)

# # Yaw 720 degrees (slow yaw for ~6 seconds)
# rc_override[3] = 1700  # Yaw right
# print("[ACTION] Yawing 720° (approx 6 seconds)...")
# start = time.time()
# while time.time() - start < 6:
#     master.mav.rc_channels_override_send(master.target_system, master.target_component, *rc_override)
#     time.sleep(1)

# # Stabilize 2s
# rc_override[3] = 1500
# print("[ACTION] Stabilizing for 2 seconds...")
# start = time.time()
# while time.time() - start < 2:
#     master.mav.rc_channels_override_send(master.target_system, master.target_component, *rc_override)
#     time.sleep(1)

# # Surface (Throttle up)
# rc_override[2] = 1400  # Come back up
# print("[ACTION] Surfacing for 2 seconds...")
# start = time.time()
# while time.time() - start < 2:
#     master.mav.rc_channels_override_send(master.target_system, master.target_component, *rc_override)
#     time.sleep(1)

# # Stop all motion
# rc_override = [1500] * 8 + [65535] * 10
# master.mav.rc_channels_override_send(master.target_system, master.target_component, *rc_override)
# print("[INFO] Motion stopped.")

# # Disarm
# print("[INFO] Disarming vehicle...")
# master.arducopter_disarm()
# master.motors_disarmed_wait()
# print("[SUCCESS] Vehicle disarmed.")


from pymavlink import mavutil
import time

# -----------------------------
# Connect to Pixhawk
# -----------------------------
print("[INFO] Connecting to Pixhawk...")
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()
print(f"[INFO] Connected to system {master.target_system}, component {master.target_component}")

# -----------------------------
# Set mode to MANUAL
# -----------------------------
mode_mapping = master.mode_mapping()
if 'MANUAL' not in mode_mapping:
    raise Exception("[ERROR] MANUAL mode not available in mode mapping.")
mode_id = mode_mapping['MANUAL']
master.set_mode(mode_id)
print("[INFO] Set to MANUAL mode.")
time.sleep(1)

# -----------------------------
# Arm the vehicle
# -----------------------------
print("[INFO] Arming the vehicle...")
master.arducopter_arm()
master.motors_armed_wait()
print("[SUCCESS] Vehicle armed.")

# -----------------------------
# RC override: 8 channels + 10 unused
# -----------------------------
# Neutral = 1500, forward/up depends on mixer mapping
rc_override = [1500] * 8 + [65535] * 10

# -----------------------------
# 1. Dive for 2 seconds (vertical thrusters)
# Assuming CH3 (index 2) is vertical thruster
# -----------------------------
rc_override[0] = 1380  # Downward thrust (dive)
rc_override[1] = 1380  # Downward thrust (dive)
print("[ACTION] Diving for 4 seconds...")
start_time = time.time()
while time.time() - start_time < 4:
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_override
    )
    time.sleep(0.1)

# -----------------------------
# 2. Idle for 5 seconds
# -----------------------------
# rc_override[2] = 1500  # Neutral vertical thrust
# print("[ACTION] Idling for 5 seconds...")
# start_time = time.time()
# while time.time() - start_time < 3:
#     master.mav.rc_channels_override_send(
#         master.target_system,
#         master.target_component,
#         *rc_override
#     )
#     time.sleep(0.1)

# -----------------------------
# 3. Yaw for 5 seconds
# -----------------------------
# 3 and 4 -> yaw
rc_override[3] = 1610
rc_override[4] = 1610
print("[ACTION] Yaw 2 times for 5 seconds...")
start_time = time.time()
while time.time() - start_time < 5:
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_override
    )
    time.sleep(1)

# -----------------------------
# 3. Roll for 8 seconds
# -----------------------------
# rc_override[1] = 1700  # Forward thrust
# rc_override[2] = 1700
# # rc_override[2] = 1700
# # rc_override[3] = 1700

# print("[ACTION] Rolling for 5 seconds...")
# start_time = time.time()
# while time.time() - start_time < 8:
#     master.mav.rc_channels_override_send(
#         master.target_system,
#         master.target_component,
#         *rc_override
#     )
#     time.sleep(0.1)

# -----------------------------
# 4. Stabilize for 3 seconds
# -----------------------------
rc_override[0] = 1500
rc_override[1] = 1500
print("[ACTION] Stabilizing for 3 seconds...")
start_time = time.time()
while time.time() - start_time < 3:
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_override
    )
    time.sleep(0.1)

# -----------------------------
# 3. Forward for 5 seconds
# -----------------------------
rc_override[1] = 1780
rc_override[0] = 1700
print("[ACTION] Upward for 10 seconds...")
start_time = time.time()
while time.time() - start_time < 5:
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_override
    )
    time.sleep(1)

# -----------------------------
# Stop all motion
# -----------------------------
# rc_override = [1500] * 8 + [65535] * 10
# master.mav.rc_channels_override_send(
#     master.target_system,
#     master.target_component,
#     *rc_override
# )
# print("[INFO] Motion stopped.")

# -----------------------------
# Disarm the vehicle
# -----------------------------
print("[INFO] Disarming the vehicle...")
master.arducopter_disarm()
master.motors_disarmed_wait()
print("[SUCCESS] Vehicle disarmed. Mission complete.")
