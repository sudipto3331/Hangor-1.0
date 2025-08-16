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
# Pressure Control Parameters
# -----------------------------
TARGET_PRESSURE = 1020.0  # hPa
print(f"[INFO] Target pressure set to {TARGET_PRESSURE} hPa")

last_pressure = None

# -----------------------------
# Run Mission: Maintain depth + move forward
# -----------------------------
print("[ACTION] Moving forward while maintaining depth for 20s...")
set_neutral()
rc_override[4] = 1600  # Forward thrust

start_time = time.time()
while time.time() - start_time < 20:
    # Read next MAVLink message
    msg = master.recv_match(type=['SCALED_PRESSURE','SCALED_PRESSURE2'], blocking=True, timeout=1)
    
    if msg:
        pressure = msg.press_abs  # Absolute pressure (hPa)
        
        if pressure != last_pressure:
            print(f"[SENSOR] Pressure: {pressure:.2f} hPa")
            last_pressure = pressure
        
        # Depth control with 1s delay after correction
        if pressure < TARGET_PRESSURE:
            # Too shallow → dive
            rc_override[0] = 1400
            rc_override[1] = 1400
            send_rc_override()
            print("[ACTION] Diving... holding for 1s")
            time.sleep(1)

        elif pressure > TARGET_PRESSURE:
            # Too deep → rise
            rc_override[0] = 1600
            rc_override[1] = 1600
            send_rc_override()
            print("[ACTION] Rising... holding for 1s")
            time.sleep(1)

        else:
            # Stable
            rc_override[0] = 1500
            rc_override[1] = 1500
            send_rc_override()
    else:
        print("[WARNING] No pressure message received!")

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
