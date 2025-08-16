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
# PID Parameters
# -----------------------------
Kp = 10.0
Ki = 0.1
Kd = 0.5

integral = 0.0
last_error = 0.0
last_time = time.time()

# -----------------------------
# Pressure Control Parameters
# -----------------------------
TARGET_PRESSURE = 1050.0  # hPa (example target)
print(f"[INFO] Target water pressure set to {TARGET_PRESSURE} hPa")

last_pressure = None

# -----------------------------
# Run Mission: Maintain depth + move forward
# -----------------------------
print("[ACTION] Moving forward while maintaining depth for 20s...")
set_neutral()
rc_override[4] = 1600  # Forward thrust

start_time = time.time()
while time.time() - start_time < 20:
    msg = master.recv_match(type=['SCALED_PRESSURE2'], blocking=True, timeout=1)
    
    if msg:
        water_pressure = msg.press_abs  # hPa
        now = time.time()
        dt = now - last_time if last_time else 0.1

        # --- PID control ---
        error = water_pressure - TARGET_PRESSURE  # flipped sign
        integral += error * dt
        derivative = (error - last_error) / dt if dt > 0 else 0.0
        output = Kp * error + Ki * integral + Kd * derivative

        pwm = 1500 + int(output)
        pwm = max(1400, min(1600, pwm))  # Clamp safe range

        rc_override[0] = pwm
        rc_override[1] = pwm
        send_rc_override()

        # Status string
        if pwm < 1500:
            status = "[DIVING]"
        elif pwm > 1500:
            status = "[RISING]"
        else:
            status = "[STABLE]"

        print(f"[SCALED_PRESSURE2] Water: {water_pressure:.2f} hPa | Target: {TARGET_PRESSURE:.2f} hPa | Error: {error:.2f} | PWM: {pwm} {status}")

        last_error = error
        last_time = now
        last_pressure = water_pressure
    else:
        print("[WARNING] No water pressure message received!")

    time.sleep(0.1)

# -----------------------------
# Mission complete - stop all motion
# -----------------------------
print("[INFO] Mission sequence complete. Stopping all motion...")
set_neutral()
send_rc_override()

# Clear RC override
rc_override = [65535] * 18
send_rc_override()
print("[INFO] RC override cleared.")

# -----------------------------
# Disarm vehicle
# -----------------------------
print("[INFO] Disarming the vehicle...")
master.arducopter_disarm()
master.motors_disarmed_wait()
print("[SUCCESS] Vehicle disarmed. Mission complete.")

# Close connection
master.close()
print("[INFO] Connection closed.")
