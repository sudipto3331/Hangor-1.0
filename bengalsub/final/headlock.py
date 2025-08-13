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

# Helper: get yaw from ATTITUDE message
def get_yaw_deg():
    msg = master.recv_match(type='ATTITUDE', blocking=True)
    yaw_rad = msg.yaw
    yaw_deg = math.degrees(yaw_rad)
    return (yaw_deg + 360) % 360  # normalize 0-360

# Helper: send RC override
rc_override = [1500]*8 + [65535]*10
def send_rc_override():
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_override
    )

# -----------------------------
# Keep heading while moving forward
# -----------------------------
target_yaw = get_yaw_deg()  # lock current heading
print(f"[INFO] Target yaw: {target_yaw:.2f}")

start_time = time.time()
while time.time() - start_time < 20:
    current_yaw = get_yaw_deg()

    error = current_yaw - target_yaw
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360

    # Simple correction: adjust yaw thruster
    yaw_pwm = 1500 - (error * 3)  # 3 PWM per degree of error
    yaw_pwm = max(min(yaw_pwm, 1600), 1400)

    # Forward thrust
    rc_override[5] = 1600  # forward thruster
    rc_override[3] = int(yaw_pwm)  # yaw control

    send_rc_override()
    time.sleep(0.1)

# Stop
rc_override = [1500]*8 + [65535]*10
send_rc_override()

# Disarm
print("[INFO] Disarming...")
master.arducopter_disarm()
master.motors_disarmed_wait()
print("[SUCCESS] Mission complete.")
