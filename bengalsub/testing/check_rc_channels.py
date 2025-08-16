from pymavlink import mavutil
import time

print("[INFO] Connecting to Pixhawk...")
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()
print(f"[INFO] Connected to system {master.target_system}, component {master.target_component}")

print("[INFO] Setting mode to STABILIZE...")
mode_mapping = master.mode_mapping()
mode_id = mode_mapping['STABILIZE']
master.set_mode(mode_id)
print("[SUCCESS] Mode set to STABILIZE")
time.sleep(2)

print("[INFO] Arming the vehicle...")
master.arducopter_arm()
master.motors_armed_wait()
print("[SUCCESS] Vehicle armed.")

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

<<<<<<< HEAD:bengalsub/final/t1.py
# -----------------------------
# Move forward and dive for 25 seconds
# -----------------------------
print("[ACTION] Moving forward and diving for 25 seconds...")
set_neutral()
rc_override[4] = 1600  # forward (pitch)
# rc_override[2] = 1510  # dive (throttle, adjust if needed)
=======
>>>>>>> 2d6d0ca821edca912febe22fcb17d533f7e976f2:bengalsub/testing/check_rc_channels.py

set_neutral()
rc_override[0] = 1500  
rc_override[1] = 1500  
rc_override[2] = 1500  
rc_override[3] = 1500  
rc_override[4] = 1500  
rc_override[5] = 1500 
rc_override[6] = 1500  
rc_override[7] = 1500  
start_time = time.time()
<<<<<<< HEAD:bengalsub/final/t1.py
while time.time() - start_time < 15:
=======
while time.time() - start_time < 10:
>>>>>>> 2d6d0ca821edca912febe22fcb17d533f7e976f2:bengalsub/testing/check_rc_channels.py
    send_rc_override()
    time.sleep(0.1)

print("[INFO] Mission sequence complete. Stopping all motion...")
set_neutral()
send_rc_override()

# Clear RC override
rc_override = [65535] * 18
send_rc_override()
print("[INFO] RC override cleared.")


print("[INFO] Disarming the vehicle...")
master.arducopter_disarm()
master.motors_disarmed_wait()
print("[SUCCESS] Vehicle disarmed. Mission complete.")

# Close connection
master.close()
print("[INFO] Connection closed.")
