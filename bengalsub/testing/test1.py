from pymavlink import mavutil
import time

TARGET_DEPTH_FT = 1.0
TARGET_DEPTH_M = TARGET_DEPTH_FT * 0.3048  # Convert to meters
TOLERANCE_M = 0.05  # ±5 cm

NEUTRAL_PWM = 1500
UP_PWM = 1520
DOWN_PWM = 1480
FORWARD_PWM = 1600

CH_VERTICAL = 3   # Adjust to match your vertical thruster channel
CH_FORWARD = 5    # Adjust to match your forward thruster channel

print("[INFO] Connecting to Pixhawk...")
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()
print(f"[INFO] Connected to system {master.target_system}, component {master.target_component}")

def set_mode(mode_name):
    """Set flight mode"""
    mode_mapping = master.mode_mapping()
    if mode_name not in mode_mapping:
        print(f"[ERROR] Mode {mode_name} not available")
        return False
    
    mode_id = mode_mapping[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    
    # Wait for mode change
    start_time = time.time()
    while time.time() - start_time < 10:  # 10 second timeout
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg:
            current_mode = master.flightmode
            if current_mode == mode_name:
                print(f"[SUCCESS] Mode changed to {mode_name}")
                return True
        time.sleep(0.5)
    
    print(f"[ERROR] Failed to change to {mode_name}")
    return False

print("[INFO] Setting ALT_HOLD mode...")
if not set_mode("ALT_HOLD"):
    print("[ERROR] Could not set ALT_HOLD mode. Exiting.")
    exit(1)

print("[INFO] Arming...")
master.arducopter_arm()
master.motors_armed_wait()
print("[SUCCESS] Armed.")

def send_rc(vertical_pwm, forward_pwm):
    rc_values = [NEUTRAL_PWM] * 8
    rc_values[CH_VERTICAL - 1] = vertical_pwm
    rc_values[CH_FORWARD - 1] = forward_pwm
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_values
    )

def get_depth():
    msg = master.recv_match(type=['SCALED_PRESSURE', 'SCALED_PRESSURE2'], blocking=True, timeout=1)
    if msg and hasattr(msg, 'press_abs'):
        pressure_pa = msg.press_abs * 100  # mbar → Pa
        depth = (pressure_pa - 101325) / (1000 * 9.80665)
        return max(depth, 0.0)
    return None

# Step 1: Dive to target depth
print(f"[ACTION] Diving to {TARGET_DEPTH_FT} ft (~{TARGET_DEPTH_M:.2f} m)...")
while True:
    depth = get_depth()
    if depth is None:
        continue
    print(f"[DEBUG] Depth: {depth:.2f} m")
    if depth < TARGET_DEPTH_M - TOLERANCE_M:
        send_rc(DOWN_PWM, NEUTRAL_PWM)
    elif depth > TARGET_DEPTH_M + TOLERANCE_M:
        send_rc(UP_PWM, NEUTRAL_PWM)
    else:
        send_rc(NEUTRAL_PWM, NEUTRAL_PWM)
        break
    time.sleep(0.1)

print("[INFO] Target depth reached.")

# Step 2: Move forward for 10s while holding depth
print("[ACTION] Moving forward while holding depth...")
start_time = time.time()
while time.time() - start_time < 10:
    depth = get_depth()
    if depth is None:
        continue
    if depth < TARGET_DEPTH_M - TOLERANCE_M:
        vertical_cmd = DOWN_PWM
    elif depth > TARGET_DEPTH_M + TOLERANCE_M:
        vertical_cmd = UP_PWM
    else:
        vertical_cmd = NEUTRAL_PWM
    send_rc(vertical_cmd, FORWARD_PWM)
    time.sleep(0.1)

# Stop thrusters
send_rc(NEUTRAL_PWM, NEUTRAL_PWM)
time.sleep(1)

# Clear RC overrides
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    *([0] * 8)  # 0 = no override
)

print("[INFO] Disarming...")
master.arducopter_disarm()
master.motors_disarmed_wait()
print("[SUCCESS] Mission complete.")
master.close()