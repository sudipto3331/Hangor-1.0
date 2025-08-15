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
# Set mode to STABILIZE
# -----------------------------
print("[INFO] Setting mode to STABILIZE...")
mode_mapping = master.mode_mapping()
mode_id = mode_mapping['STABILIZE']
master.set_mode(mode_id)
print("[SUCCESS] Mode set to STABILIZE")
time.sleep(2)

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

# -----------------------------
# Move forward and dive gradually
# -----------------------------
print("[ACTION] Moving forward and diving gradually for 25 seconds...")
set_neutral()
rc_override[4] = 1600  # forward (pitch)

start_time = time.time()
dive_pwm = 1520  # initial dive PWM (less than 1500 to go down)

while time.time() - start_time < 25:
    # Pulse dive every 1 second
    current_time = time.time() - start_time
    if int(current_time) % 2 == 0:
        rc_override[2] = dive_pwm  # dive
    else:
        rc_override[2] = 1520  # neutral throttle
    
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
