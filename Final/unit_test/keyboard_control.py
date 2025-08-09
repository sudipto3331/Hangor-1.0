from pymavlink import mavutil
from pynput import keyboard
import time

print("[INFO] Connecting to Pixhawk...")
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()
print(f"[INFO] Connected to system {master.target_system}, component {master.target_component}")

# Set mode to MANUAL (or another mode that allows RC override)
mode_mapping = master.mode_mapping()
if 'MANUAL' not in mode_mapping:
    raise Exception("[ERROR] MANUAL mode not available.")
mode_id = mode_mapping['MANUAL']
master.set_mode(mode_id)
print("[INFO] Set to MANUAL mode.")
time.sleep(1)

# Arm the vehicle
print("[INFO] Arming the vehicle...")
master.arducopter_arm()
master.motors_armed_wait()
print("[INFO] Vehicle armed successfully.")

# PWM values
PWM_NEUTRAL = 1500
PWM_FORWARD = 1600
PWM_BACKWARD = 1400
PWM_LEFT = 1400
PWM_RIGHT = 1600
PWM_UP = 1600
PWM_DOWN = 1400

# Channels (adjust for your setup)
rc_override = [PWM_NEUTRAL] * 8 + [65535] * 10

def send_rc():
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_override
    )

def move_forward():
    rc_override[4] = PWM_FORWARD
    rc_override[5] = PWM_FORWARD
    send_rc()

def move_backward():
    rc_override[4] = PWM_BACKWARD
    rc_override[5] = PWM_BACKWARD
    send_rc()

def move_left():
    rc_override[6] = PWM_LEFT
    rc_override[7] = PWM_RIGHT
    send_rc()

def move_right():
    rc_override[6] = PWM_RIGHT
    rc_override[7] = PWM_LEFT
    send_rc()

def move_up():
    rc_override[2] = PWM_UP
    rc_override[3] = PWM_UP
    send_rc()

def move_down():
    rc_override[2] = PWM_DOWN
    rc_override[3] = PWM_DOWN
    send_rc()

def stop_all():
    for i in range(8):
        rc_override[i] = PWM_NEUTRAL
    send_rc()

def on_press(key):
    try:
        if key.char == 'w':
            move_forward()
        elif key.char == 's':
            move_backward()
        elif key.char == 'a':
            move_left()
        elif key.char == 'd':
            move_right()
        elif key.char == 'r':
            move_up()
        elif key.char == 'q':
            move_down()
    except AttributeError:
        pass

def on_release(key):
    stop_all()
    if key == keyboard.Key.esc:
        print("[INFO] Disarming the vehicle...")
        master.arducopter_disarm()
        master.motors_disarmed_wait()
        print("[SUCCESS] Vehicle disarmed.")
        return False

print("[INFO] Controls: W/S/A/D = move, R = up, Q = down, ESC = quit.")
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()
