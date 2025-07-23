import tkinter as tk
from pymavlink import mavutil
import time
import threading
import subprocess
import os
from functools import partial

# Connect to Pixhawk via MAVLink (UDP port)
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait for heartbeat to ensure connection
master.wait_heartbeat()

print(f"[INFO] Connected to system {master.target_system}, component {master.target_component}")

# RC override array: 8 main + 10 spare
rc_override = [1500] * 8 + [65535] * 10
rc_lock = threading.Lock()


# Tkinter GUI setup
root = tk.Tk()
root.title("Thruster Control GUI – BlueROV2 Heavy")

battery_voltage = tk.StringVar()
battery_current = tk.StringVar()
battery_voltage.set("Voltage: --- V")
battery_current.set("Current: --- A")

# Background loop: heartbeat + RC + battery telemetry
def background_loop():
    while True:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        with rc_lock:
            master.mav.rc_channels_override_send(
                master.target_system,
                master.target_component,
                *rc_override
            )

        msg = master.recv_match(type='BATTERY_STATUS', blocking=False)
        if msg:
            v = msg.voltages[0]
            c = msg.current_battery
            if v != 0xFFFF:
                battery_voltage.set(f"Voltage: {v / 1000:.2f} V")
            if c != -1:
                battery_current.set(f"Current: {c / 100.0:.2f} A")
        time.sleep(0.2)

threading.Thread(target=background_loop, daemon=True).start()

# Arm / disarm commands
def arm():
    print("[INFO] Arming...")
    if 'MANUAL' in master.mode_mapping():
        mode_id = master.mode_mapping()['MANUAL']
        master.set_mode(mode_id)
        time.sleep(1)
    master.arducopter_arm()
    try:
        master.motors_armed_wait()
        print("[SUCCESS] Armed.")
    except:
        print("[ERROR] Arming failed.")

def disarm():
    print("[INFO] Disarming...")
    master.arducopter_disarm()
    try:
        master.motors_disarmed_wait()
        print("[SUCCESS] Disarmed.")
    except:
        print("[ERROR] Disarming failed.")

# RC override update
def update_thruster(index, value):
    with rc_lock:
        rc_override[index] = int(value)

# Fix: must accept `event` as first argument
def on_release(event, index, slider):
    slider.set(1500)
    update_thruster(index, 1500)

# Launch joystick GUI
def launch_joystick_gui():
    path = os.path.join(os.path.dirname(__file__), "joy.py")
    try:
        subprocess.Popen(["python3", path])
        print("[INFO] Joystick GUI launched.")
    except Exception as e:
        print(f"[ERROR] {e}")

# Top buttons
tk.Button(root, text="ARM", font=("Arial", 12), bg="green", fg="white", command=arm).pack(pady=4)
tk.Button(root, text="DISARM", font=("Arial", 12), bg="red", fg="white", command=disarm).pack(pady=4)
tk.Button(root, text="Joystick Test", font=("Arial", 11), bg="blue", fg="white", command=launch_joystick_gui).pack(pady=4)

tk.Label(root, textvariable=battery_voltage, font=("Arial", 12)).pack()
tk.Label(root, textvariable=battery_current, font=("Arial", 12)).pack()

# 8 sliders for MAIN OUT 1–8
for i in range(8):
    frame = tk.Frame(root)
    frame.pack(pady=5)
    tk.Label(frame, text=f"Thruster {i+1} (MAIN OUT {i+1})", font=("Arial", 11)).pack(side=tk.LEFT, padx=5)

    slider = tk.Scale(frame, from_=1100, to=1900, orient=tk.HORIZONTAL,
                      length=300, resolution=1)
    slider.set(1500)
    slider.pack(side=tk.LEFT)

    # Bind slider to correct thruster
    slider.config(command=partial(update_thruster, i))
    slider.bind("<ButtonRelease-1>", partial(on_release, index=i, slider=slider))

print("[READY] GUI running. Slide to control thrusters, auto reset on release.")
root.mainloop()
