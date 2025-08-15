from pymavlink import mavutil
import time

# -----------------------------
# Connect to Pixhawk
# -----------------------------
print("[INFO] Connecting to Pixhawk...")
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()
print(f"[INFO] Connected to system {master.target_system}")

# Set mode to STABILIZE
master.set_mode(master.mode_mapping()['STABILIZE'])
print("[SUCCESS] Mode set to STABILIZE")

# Arm the vehicle
master.arducopter_arm()
master.motors_armed_wait()
print("[SUCCESS] Vehicle armed")

# -----------------------------
# Depth variables
# -----------------------------
surface_pressure = None
target_depth = 0.5

# RC override
rc_override = [1500] * 8 + [65535] * 10

def send_rc_override():
    master.mav.rc_channels_override_send(master.target_system, master.target_component, *rc_override)

def set_neutral():
    for i in range(8):
        rc_override[i] = 1500

def get_depth():
    """Get depth from MS5611 pressure sensor"""
    global surface_pressure
    
    msg = master.recv_match(type='SCALED_PRESSURE', blocking=False, timeout=0.1)
    if msg:
        current_pressure = msg.press_abs  # mbar
        
        # Calibrate surface pressure on first reading
        if surface_pressure is None:
            surface_pressure = current_pressure
            print(f"[INFO] Surface pressure: {surface_pressure:.1f} mbar")
            return 0
        
        # Calculate depth: 1 mbar ≈ 1 cm depth
        pressure_diff = current_pressure - surface_pressure
        depth = pressure_diff / 100.0  # Convert to meters
        return max(0, depth)
    
    return 0

def hold_depth(target, duration=60):
    """Simple depth holding using basic control"""
    print(f"[INFO] Holding depth at {target:.2f}m for {duration}s")
    
    start_time = time.time()
    while time.time() - start_time < duration:
        current_depth = get_depth()
        error = target - current_depth
        
        # Simple proportional control
        if error > 0.05:  # Need to go deeper
            rc_override[2] = 1600  # Down thrust
        elif error < -0.05:  # Need to go up
            rc_override[2] = 1400  # Up thrust
        else:
            rc_override[2] = 1500  # Neutral
        
        print(f"[DEPTH] Current: {current_depth:.2f}m, Target: {target:.2f}m")
        send_rc_override()
        time.sleep(0.2)

# -----------------------------
# MISSION START
# -----------------------------
print("[INFO] Calibrating surface pressure...")
get_depth()  # Calibrate surface
time.sleep(2)

# 1. DIVE for 15 seconds
print("[ACTION] Diving for 10 seconds...")
set_neutral()
rc_override[0] = 1430  # Forward
rc_override[1] = 1430  # Forward

start_time = time.time()
while time.time() - start_time < 10:
    depth = get_depth()
    print(f"[DIVE] Depth: {depth:.2f}m")
    send_rc_override()
    time.sleep(0.5)

# 2. HOLD DEPTH
final_depth = get_depth()
print(f"[INFO] Dive complete at {final_depth:.2f}m")

set_neutral()
hold_depth(final_depth, 60)  # Hold for 60 seconds

# 3. FINISH
print("[INFO] Mission complete")
set_neutral()
send_rc_override()
master.arducopter_disarm()
print("[SUCCESS] Vehicle disarmed")