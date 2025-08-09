from pymavlink import mavutil
from pynput import keyboard
import time

# --- MAVLink connection ---
print("[INFO] Connecting to Pixhawk...")
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()
print(f"[INFO] Connected to system {master.target_system}, component {master.target_component}")

# Arm vehicle
print("[INFO] Arming the vehicle...")
master.arducopter_arm()
master.motors_armed_wait()
print("[INFO] Vehicle armed successfully.")

PWM_NEUTRAL = 1500
PWM_FORWARD = 1600
NUM_THRUSTERS = 8

# RC override array: 8 channels + 10 spares
rc_override = [PWM_NEUTRAL] * 8 + [65535] * 10

def send_rc():
    print(f"[DEBUG] Sending RC override: {rc_override}")  # Debug print
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_override
    )

def run_thruster(thruster_num):
    # thruster_num: 1-8, maps to rc_override index 0-7
    idx = thruster_num - 1
    rc_override[idx] = PWM_FORWARD
    print(f"[DEBUG] Thruster {thruster_num} set to forward (PWM: {PWM_FORWARD})")  # Debug print
    send_rc()
    print(f"[ACTION] Thruster {thruster_num} running.")

def stop_thruster(thruster_num):
    idx = thruster_num - 1
    rc_override[idx] = PWM_NEUTRAL
    print(f"[DEBUG] Thruster {thruster_num} set to neutral (PWM: {PWM_NEUTRAL})")  # Debug print
    send_rc()
    print(f"[INFO] Thruster {thruster_num} stopped.")

def stop_all():
    for i in range(NUM_THRUSTERS):
        rc_override[i] = PWM_NEUTRAL
    send_rc()
    print("[INFO] All thrusters stopped.")

# --- Keyboard event handlers ---
pressed_thrusters = set()

def on_press(key):
    try:
        if hasattr(key, 'char') and key.char in '12345678':  # Ensure key has a 'char' attribute
            thruster = int(key.char)
            if thruster not in pressed_thrusters:
                run_thruster(thruster)
                pressed_thrusters.add(thruster)
                print(f"[INFO] Key {key.char} pressed: Thruster {thruster} running.")  # Debug print
    except AttributeError:
        pass

def on_release(key):
    try:
        if hasattr(key, 'char') and key.char in '12345678':  # Ensure key has a 'char' attribute
            thruster = int(key.char)
            if thruster in pressed_thrusters:
                stop_thruster(thruster)
                pressed_thrusters.remove(thruster)
                print(f"[INFO] Key {key.char} released: Thruster {thruster} stopped.")  # Debug print
    except AttributeError:
        pass

    if key == keyboard.Key.esc:
        print("[INFO] Disarming and exiting...")
        stop_all()
        master.arducopter_disarm()
        master.motors_disarmed_wait()
        print("[SUCCESS] Vehicle disarmed.")
        return False  # Exit listener

print("[INFO] Press keys 1-8 to test thrusters. Release to stop. ESC to exit.")

# Start the keyboard listener
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()
