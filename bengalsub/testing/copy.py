#!/usr/bin/env python3
"""
AUV Depth-Hold and Forward-Move Script
- Uses Bar30 pressure sensor with Pixhawk
- Depth control via RC override
- Moves forward while maintaining target depth
"""

from pymavlink import mavutil
import time

# -----------------------------
# CONFIGURATION
# -----------------------------
TARGET_DEPTH_FT = 1.0
TARGET_DEPTH_M = TARGET_DEPTH_FT * 0.3048  # Convert to meters
TOLERANCE_M = 0.05  # ±5 cm depth band

# PWM values (adjust for your ESC calibration)
NEUTRAL_PWM = 1500
UP_PWM = 1520        # Thruster power to ascend
DOWN_PWM = 1480      # Thruster power to descend
FORWARD_PWM = 1600   # Forward movement power

# RC channel mapping (check in QGC/Mission Planner)
CH_VERTICAL = 3   # Vertical thruster
CH_FORWARD = 5    # Forward thruster

# Forward travel time
FORWARD_TIME_SEC = 10

# -----------------------------
# CONNECT TO PIXHAWK
# -----------------------------
print("[INFO] Connecting to Pixhawk...")
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()
print(f"[INFO] Connected to system {master.target_system}, component {master.target_component}")

# -----------------------------
# ARM VEHICLE
# -----------------------------
print("[INFO] Arming thrusters...")
master.arducopter_arm()
master.motors_armed_wait()
print("[SUCCESS] Armed.")

# -----------------------------
# FUNCTION DEFINITIONS
# -----------------------------
def send_rc(vertical_pwm, forward_pwm):
    """
    Sends RC override commands to the Pixhawk.
    """
    rc_values = [NEUTRAL_PWM] * 8
    rc_values[CH_VERTICAL - 1] = vertical_pwm
    rc_values[CH_FORWARD - 1] = forward_pwm
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_values
    )

def get_depth():
    """
    Reads depth from the Bar30 pressure sensor.
    Returns depth in meters, or None if no data.
    """
    msg = master.recv_match(
        type=['SCALED_PRESSURE', 'SCALED_PRESSURE2'],
        blocking=True, timeout=1
    )
    if msg and hasattr(msg, 'press_abs'):
        pressure_pa = msg.press_abs * 100  # mbar → Pa
        depth_m = (pressure_pa - 101325) / (1000 * 9.80665)  # Freshwater calc
        return max(depth_m, 0.0)
    return None

# -----------------------------
# STEP 1: Dive to Target Depth
# -----------------------------
print(f"[ACTION] Diving to {TARGET_DEPTH_FT:.1f} ft (~{TARGET_DEPTH_M:.2f} m)...")
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

# -----------------------------
# STEP 2: Hold Depth & Move Forward
# -----------------------------
print(f"[ACTION] Moving forward for {FORWARD_TIME_SEC}s while holding depth...")
start_time = time.time()
while time.time() - start_time < FORWARD_TIME_SEC:
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

# -----------------------------
# STOP & DISARM
# -----------------------------
send_rc(NEUTRAL_PWM, NEUTRAL_PWM)
time.sleep(1)

print("[INFO] Disarming...")
master.arducopter_disarm()
master.motors_disarmed_wait()
print("[SUCCESS] Mission complete.")
master.close()
