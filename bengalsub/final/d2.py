from pymavlink import mavutil
import time

# -----------------------------
# User target depth (in feet)
# -----------------------------
target_depth_ft = 1
target_depth = target_depth_ft * 0.3048  # Convert to meters

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
# RC override setup
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
# Function to read depth (meters)
# -----------------------------
def read_depth():
    msg = master.recv_match(type=['SCALED_PRESSURE', 'SCALED_PRESSURE2'], blocking=True, timeout=1)
    if msg:
        pressure_mbar = msg.press_abs  # Absolute pressure in mbar
        depth_m = (pressure_mbar - 1013.25) / 100.0  # Approx for freshwater
        return depth_m
    return None

# -----------------------------
# Move forward while holding target depth
# -----------------------------
print(f"[ACTION] Moving forward at 1600 PWM while holding depth {target_depth_ft:.2f} ft ({target_depth:.2f} m)")

set_neutral()
rc_override[4] = 1600  # Forward thrust

start_time = time.time()
while time.time() - start_time < 20:  # Run for 20 seconds
    current_depth = read_depth()
    if current_depth is not None:
        depth_error = current_depth - target_depth

        # Simple proportional control
        if depth_error < -0.05:  # Too shallow, go down
            rc_override[2] = 1600
        elif depth_error > 0.05:  # Too deep, go up
            rc_override[2] = 1400
        else:  # Within range
            rc_override[2] = 1500

        print(f"Depth: {current_depth:.2f} m  |  Target: {target_depth:.2f} m  |  Error: {depth_error:.2f}")

    send_rc_override()
    time.sleep(0.1)

# -----------------------------
# Stop all motion
# -----------------------------
print("[INFO] Mission complete. Stopping all motion...")
set_neutral()
send_rc_override()

rc_override = [65535] * 18
send_rc_override()

# -----------------------------
# Disarm
# -----------------------------
print("[INFO] Disarming...")
master.arducopter_disarm()
master.motors_disarmed_wait()
print("[SUCCESS] Vehicle disarmed. Mission complete.")

master.close()
