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
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_override
    )

def set_neutral():
    for i in range(8):
        rc_override[i] = 1500

# -----------------------------
# Depth hold parameters
# -----------------------------
TARGET_PRESSURE = 1058.00  # hPa
DEPTH_KP = 2.0             # proportional gain for throttle control

# -----------------------------
# Move forward and hold depth
# -----------------------------
print("[ACTION] Moving forward and holding depth at 1049 hPa for 25 seconds...")
set_neutral()
rc_override[4] = 1600  # forward (pitch)

start_time = time.time()
while time.time() - start_time < 25:
    # Read the latest SCALED_PRESSURE2 MAVLink message
    msg = master.recv_match(type='SCALED_PRESSURE2', blocking=False)
    if msg is not None:
        current_pressure = msg.press_abs / 100.0  # convert Pa to hPa
        error = TARGET_PRESSURE - current_pressure

        # proportional control for throttle
        throttle_pwm = int(1500 + DEPTH_KP * error)
        throttle_pwm = max(1400, min(1600, throttle_pwm))  # clamp

        rc_override[2] = throttle_pwm  # set throttle

        print(f"[DEBUG] Pressure: {current_pressure:.2f} hPa, Throttle: {throttle_pwm}")

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
