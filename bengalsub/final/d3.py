from pymavlink import mavutil
import time

# -----------------------------
# USER SETTINGS
# -----------------------------
TARGET_DEPTH_METERS = 0.305  # 1 ft
FORWARD_SPEED = 0.3          # Range: -1.0 (full reverse) to +1.0 (full forward)
MOVE_DURATION = 30           # seconds to move forward

# -----------------------------
# CONNECT TO PIXHAWK
# -----------------------------
print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()
print(f"Heartbeat from system {master.target_system} component {master.target_component}")

# -----------------------------
# SET MODE: DEPTH HOLD
# -----------------------------
def set_mode(mode_name):
    mode_id = master.mode_mapping()[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

print("Setting mode to DEPTH_HOLD...")
set_mode("ALT_HOLD")  # Depth hold is ALT_HOLD in ArduSub
time.sleep(1)

# -----------------------------
# ARM MOTORS
# -----------------------------
print("Arming AUV...")
master.arducopter_arm()
master.motors_armed_wait()
print("AUV armed!")

# -----------------------------
# SET TARGET DEPTH
# -----------------------------
def set_target_depth(depth_m):
    master.mav.set_position_target_global_int_send(
        0,                          # time_boot_ms
        master.target_system,       # target_system
        master.target_component,    # target_component
        mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        0b110111111000,              # type_mask: ignore lat/lon/yaw, only use Z
        0, 0,                        # lat_int, lon_int
        depth_m, 0, 0,                # alt (negative for depth), vx, vy
        0, 0, 0,                      # afx, afy, afz
        0, 0                          # yaw, yaw_rate
    )

# -----------------------------
# SEND MANUAL_CONTROL FOR FORWARD MOVEMENT
# -----------------------------
def send_forward(speed):
    # X: forward/back, Y: left/right, Z: up/down (throttle), R: rotation
    master.mav.manual_control_send(
        master.target_system,
        int(speed * 1000),  # X axis (forward)
        0,                  # Y axis
        500,                # Z axis (500 is neutral throttle)
        0,                  # R axis
        0                   # Buttons
    )

print(f"Holding depth {TARGET_DEPTH_METERS} m and moving forward for {MOVE_DURATION} seconds...")
start_time = time.time()
while time.time() - start_time < MOVE_DURATION:
    set_target_depth(-TARGET_DEPTH_METERS)  # Depth in meters (negative down)
    send_forward(FORWARD_SPEED)
    time.sleep(0.1)  # Send at ~10 Hz

# -----------------------------
# STOP AND DISARM
# -----------------------------
print("Stopping AUV...")
send_forward(0)
time.sleep(1)
print("Disarming...")
master.arducopter_disarm()
master.motors_disarmed_wait()
print("AUV disarmed.")
