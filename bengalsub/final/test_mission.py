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
rc_override = [1500] * 8 + [65535] * 10

def send_rc_override():
    """Send the current RC override values"""
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_override
    )

def set_neutral():
    """Set all channels to neutral position"""
    for i in range(8):
        rc_override[i] = 1500

# # -----------------------------
# # MISSION SEQUENCE START
# # -----------------------------
# 1. DIVING (DOWN) for 3s at 1700 PWM
print("[ACTION] 1. Diving DOWN for 4 seconds.")
set_neutral()
rc_override[0] = 1380
rc_override[1] = 1380
# rc_override[2] = 1380
# rc_override[3] = 1380
start_time = time.time()
while time.time() - start_time < 4:
    send_rc_override()
    time.sleep(0.1)

# 2. REMAIN IDLE for 2s
print("[ACTION] 2. Remaining idle for 2 seconds...")
set_neutral()
start_time = time.time()
while time.time() - start_time < 2:
    send_rc_override()
    time.sleep(0.1)

# 3. FORWARD for 5s at 1600 PWM
print("[ACTION] 3. Moving forward for 10 seconds at 1600 PWM...")
set_neutral()
rc_override[4] = 1600
rc_override[5] = 1600
start_time = time.time()
while time.time() - start_time < 8:
    send_rc_override()
    time.sleep(0.1)

# 4. REMAIN IDLE for 2s
print("[ACTION] 4. Remaining idle for 2 seconds...")
set_neutral()
start_time = time.time()
while time.time() - start_time < 2:
    send_rc_override()
    time.sleep(0.1)

# 5. PERFORM YAW 720 degrees at 1700 PWM
# Assuming 720° takes about 6 seconds at 1700 PWM (adjust timing as needed)
print("[ACTION] 5. Performing YAW 720° for 6 seconds at 1700 PWM...")
set_neutral()
rc_override[3] = 1650  
rc_override[4] = 1650  
start_time = time.time()
while time.time() - start_time < 5:
    send_rc_override()
    time.sleep(0.5)

# 6. COME TO SURFACE (UP) for 5s at 1700 PWM
print("[ACTION] 6. Surfacing UP for 5 seconds at 1700 PWM...")
set_neutral()
rc_override[0] = 1720
rc_override[1] = 1720
# rc_override[2] = 1620
# rc_override[3] = 1620
start_time = time.time()
while time.time() - start_time < 5:
    send_rc_override()
    time.sleep(0.1)

# -----------------------------
# MISSION COMPLETE - STOP ALL MOTION
# -----------------------------
print("[INFO] Mission sequence complete. Stopping all motion...")
set_neutral()
send_rc_override()

# Clear RC override
rc_override = [65535] * 18
send_rc_override()
print("[INFO] RC override cleared.")

# -----------------------------
# Disarm the vehicle
# -----------------------------
print("[INFO] Disarming the vehicle...")
master.arducopter_disarm()
master.motors_disarmed_wait()
print("[SUCCESS] Vehicle disarmed. Mission complete.")

# Close connection
master.close()
print("[INFO] Connection closed.")