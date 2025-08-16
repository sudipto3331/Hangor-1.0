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
TARGET_PRESSURE = 1050.0  # hPa
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
    # Read water pressure only
    msg = master.recv_match(type=['SCALED_PRESSURE2'], blocking=True, timeout=1)
    
    if msg:
        water_pressure = msg.press_abs  # hPa
        
        if water_pressure != last_pressure:
            print(f"[SCALED_PRESSURE2] Water: {water_pressure:.2f} hPa | Target: {TARGET_PRESSURE:.2f} hPa")
            last_pressure = water_pressure

            if water_pressure < TARGET_PRESSURE:
                rc_override[0] = 1450
                rc_override[1] = 1450
                send_rc_override()
                print("[ACTION] Diving... hold 1s")
                time.sleep(0.1)

            elif water_pressure > TARGET_PRESSURE:
                rc_override[0] = 1550
                rc_override[1] = 1550
                send_rc_override()
                print("[ACTION] Rising... hold 1s")
                time.sleep(0.1)

            else:
                rc_override[0] = 1500
                rc_override[1] = 1500
                send_rc_override()
                print("[ACTION] Stable")

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
