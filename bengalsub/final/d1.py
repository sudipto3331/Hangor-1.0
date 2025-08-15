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
# Function to read depth (meters)
# -----------------------------
def read_depth():
    msg = master.recv_match(type=['SCALED_PRESSURE', 'SCALED_PRESSURE2'], blocking=True, timeout=1)
    if msg:
        # Convert pressure (mbar) to depth (approx)
        # freshwater: depth(m) ≈ (pressure_mbar - 1013.25) / 100
        pressure_mbar = msg.press_abs
        depth = (pressure_mbar - 1013.25) / 100.0
        return depth
    return None

# -----------------------------
# Store initial depth
# -----------------------------
print("[INFO] Waiting for initial depth reading...")
target_depth = None
while target_depth is None:
    d = read_depth()
    if d is not None:
        target_depth = d
        print(f"[INFO] Target depth locked at {target_depth:.2f} m")

# -----------------------------
# Move forward and hold depth
# -----------------------------
print("[ACTION] Moving forward at 1600 PWM while holding depth...")

set_neutral()
rc_override[4] = 1600  # Forward thrust

start_time = time.time()
while time.time() - start_time < 20:
    current_depth = read_depth()
    if current_depth is not None:
        # Simple proportional adjustment (no PID)
        if current_depth < target_depth - 0.05:  # Too shallow
            rc_override[2] = 1600  # Down
        elif current_depth > target_depth + 0.05:  # Too deep
            rc_override[2] = 1400  # Up
        else:
            rc_override[2] = 1500  # Neutral
        
    send_rc_override()
    time.sleep(0.1)

# -----------------------------
# Stop motion
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
print("[SUCCESS] Disarmed. Mission complete.")

master.close()
