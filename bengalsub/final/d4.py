from pymavlink import mavutil
import time

# -----------------------------
# CONFIG
# -----------------------------
TARGET_DEPTH_FT = 1.0
TARGET_DEPTH_M = TARGET_DEPTH_FT * 0.3048
FORWARD_PWM = 1600
MOVE_DURATION = 20  # seconds

# RC channel indices in rc_override:
#   Ch5 (index 4) is forward/reverse in BlueROV2 Heavy by default
FORWARD_CHANNEL_INDEX = 4

# -----------------------------
# Connect to Pixhawk
# -----------------------------
print("[INFO] Connecting to Pixhawk...")
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()
print(f"[INFO] Connected to system {master.target_system}, component {master.target_component}")

# -----------------------------
# List available modes
# -----------------------------
mode_mapping = master.mode_mapping()
print("[INFO] Available modes:", list(mode_mapping.keys()))

# -----------------------------
# Set mode to DEPTH_HOLD if available
# -----------------------------
if "DEPTH_HOLD" in mode_mapping:
    print("[INFO] Setting mode to DEPTH_HOLD...")
    mode_id = mode_mapping["DEPTH_HOLD"]
    master.set_mode(mode_id)
    print("[SUCCESS] Mode set to DEPTH_HOLD")
else:
    print("[WARNING] DEPTH_HOLD mode not found! Falling back to MANUAL.")
    if "MANUAL" in mode_mapping:
        mode_id = mode_mapping["MANUAL"]
        master.set_mode(mode_id)
        print("[SUCCESS] Mode set to MANUAL")
    else:
        raise RuntimeError("Neither DEPTH_HOLD nor MANUAL modes are available.")

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
# Monitor depth from VFR_HUD
# -----------------------------
def get_depth_m():
    msg = master.recv_match(type="VFR_HUD", blocking=False)
    if msg:
        # msg.alt is negative when underwater in ArduSub
        return -msg.alt
    return None

# -----------------------------
# Move forward while holding depth
# -----------------------------
print(f"[ACTION] Moving forward for {MOVE_DURATION} seconds at {FORWARD_PWM} PWM and holding depth at {TARGET_DEPTH_FT} ft...")
set_neutral()
rc_override[FORWARD_CHANNEL_INDEX] = FORWARD_PWM

start_time = time.time()
while time.time() - start_time < MOVE_DURATION:
    depth = get_depth_m()
    if depth is not None:
        print(f"[DEPTH] Current depth: {depth:.2f} m")
    send_rc_override()
    time.sleep(0.1)

# -----------------------------
# STOP ALL MOTION
# -----------------------------
print("[INFO] Mission complete. Stopping motion...")
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
print("[INFO] Connection closed.")
