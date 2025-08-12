from pymavlink import mavutil
import time
import math

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
# RC override setup
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
    send_rc_override()

# -----------------------------
# Sensor helpers
# -----------------------------
def get_depth():
    """
    Reads depth from barometer (VFR_HUD.alt is altitude above home in meters).
    For underwater use with pressure sensor, you might need SCALED_PRESSURE2 conversion.
    Here we just invert altitude for 'depth'.
    """
    msg = master.recv_match(type='VFR_HUD', blocking=False)
    if msg:
        return -msg.alt  # Negative altitude means depth (m)
    return None

def get_yaw():
    """Reads yaw from ATTITUDE in degrees [0..360)"""
    msg = master.recv_match(type='ATTITUDE', blocking=False)
    if msg:
        yaw_deg = math.degrees(msg.yaw) % 360
        return yaw_deg
    return None

def yaw_difference(target, current):
    """Smallest signed yaw difference in degrees"""
    diff = (target - current + 180) % 360 - 180
    return diff

# -----------------------------
# Movement helpers
# -----------------------------
def dive_to_depth(target_depth_m, pwm=1380, tolerance=0.1):
    """Dive until reaching target depth using barometer"""
    print(f"[ACTION] Diving to {target_depth_m:.2f} m...")
    set_neutral()
    rc_override[0] = pwm
    rc_override[1] = pwm
    while True:
        depth = get_depth()
        if depth is not None:
            if depth >= target_depth_m - tolerance:
                break
        send_rc_override()
        time.sleep(0.1)
    set_neutral()

def surface_to_depth(target_depth_m=0.0, pwm=1700, tolerance=0.1):
    """Surface until reaching target depth"""
    print(f"[ACTION] Surfacing to {target_depth_m:.2f} m...")
    set_neutral()
    rc_override[0] = pwm
    rc_override[1] = pwm
    while True:
        depth = get_depth()
        if depth is not None:
            if depth <= target_depth_m + tolerance:
                break
        send_rc_override()
        time.sleep(0.1)
    set_neutral()

def forward_for_time(seconds, pwm=1600):
    """Move forward for a fixed time"""
    print(f"[ACTION] Moving forward for {seconds} s at PWM {pwm}...")
    set_neutral()
    rc_override[2] = pwm
    rc_override[3] = pwm
    start = time.time()
    while time.time() - start < seconds:
        send_rc_override()
        time.sleep(0.1)
    set_neutral()

def yaw_by_degrees(delta_deg, pwm=1700):
    """Rotate by given degrees using yaw feedback"""
    print(f"[ACTION] Yawing {delta_deg}°...")
    start_yaw = get_yaw()
    if start_yaw is None:
        print("[WARN] No yaw data, using timed rotation fallback.")
        # fallback: timed spin
        set_neutral()
        rc_override[3] = pwm
        rc_override[4] = pwm
        time.sleep(abs(delta_deg) / 120.0)  # rough guess: 120°/sec
        set_neutral()
        return
    target_yaw = (start_yaw + delta_deg) % 360
    set_neutral()
    # direction: positive delta -> clockwise
    direction = 1 if delta_deg > 0 else -1
    if direction > 0:
        rc_override[3] = pwm
        rc_override[4] = pwm
    else:
        rc_override[3] = 3000 - pwm  # reverse PWM
        rc_override[4] = 3000 - pwm
    while True:
        current_yaw = get_yaw()
        if current_yaw is not None:
            if abs(yaw_difference(target_yaw, current_yaw)) <= 2:
                break
        send_rc_override()
        time.sleep(0.05)
    set_neutral()

# -----------------------------
# Mission sequence with sensors
# -----------------------------
dive_to_depth(2.0)            # Dive to 2 meters depth
forward_for_time(15, 1600)     # Forward for 15s
yaw_by_degrees(720)           # Rotate twice (720°)
surface_to_depth(0.0)         # Surface

# -----------------------------
# Disarm
# -----------------------------
print("[INFO] Mission complete. Disarming...")
master.arducopter_disarm()
master.motors_disarmed_wait()
print("[SUCCESS] Disarmed.")

# Close
master.close()
print("[INFO] Connection closed.")
