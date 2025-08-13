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

# Arm the vehicle
print("[INFO] Arming the vehicle...")
master.arducopter_arm()
master.motors_armed_wait()
print("[SUCCESS] Vehicle armed.")

# -----------------------------
# Helpers
# -----------------------------
rc_override = [1500]*8 + [65535]*10

def send_rc_override():
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_override
    )

def get_yaw_deg():
    msg = master.recv_match(type='ATTITUDE', blocking=True)
    yaw_rad = msg.yaw
    return (math.degrees(yaw_rad) + 360) % 360

def get_depth_m():
    """Read depth from VFR_HUD (alt in meters below surface if negative)"""
    msg = master.recv_match(type='VFR_HUD', blocking=True, timeout=1)
    if msg:
        # For underwater vehicles, alt is often negative; convert to positive depth
        return -msg.alt
    return None

# -----------------------------
# Lock heading & depth
# -----------------------------
target_yaw = get_yaw_deg()
target_depth = get_depth_m()
if target_depth is None:
    target_depth = 0.5  # fallback: 0.5 m
print(f"[INFO] Target yaw: {target_yaw:.2f}°, target depth: {target_depth:.2f} m")

# -----------------------------
# Forward motion while keeping heading & depth
# -----------------------------
start_time = time.time()
while time.time() - start_time < 8:
    # Get current yaw & depth
    current_yaw = get_yaw_deg()
    current_depth = get_depth_m() or target_depth

    # ----- Yaw correction -----
    error_yaw = current_yaw - target_yaw
    if error_yaw > 180:
        error_yaw -= 360
    elif error_yaw < -180:
        error_yaw += 360
    yaw_pwm = 1500 - (error_yaw * 3)  # adjust gain if too aggressive
    yaw_pwm = max(min(yaw_pwm, 1600), 1400)

    # ----- Depth correction -----
    error_depth = current_depth - target_depth
    depth_pwm = 1500 - (error_depth * 50)  # 50 PWM per meter
    depth_pwm = max(min(depth_pwm, 1700), 1300)

    # ----- Send RC commands -----
    rc_override[0] = depth_pwm  # vertical thruster 1
    rc_override[1] = depth_pwm  # vertical thruster 2
    rc_override[4] = 1600       # forward thruster
    rc_override[3] = int(yaw_pwm)  # yaw control

    send_rc_override()
    time.sleep(0.1)

# Stop thrusters
rc_override = [1500]*8 + [65535]*10
send_rc_override()

# Disarm
print("[INFO] Disarming...")
master.arducopter_disarm()
master.motors_disarmed_wait()
print("[SUCCESS] Mission complete.")
