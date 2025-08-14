from pymavlink import mavutil
import time

# Connect to Pixhawk UDP endpoint (avoid conflict with QGC)
# master = mavutil.mavlink_connection('udpout:192.168.2.1:14550')
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()
print("Connected to Pixhawk")

# Surface pressure in hPa (measure once in air or use 1013.25)
surface_pressure = 1013.25

# Desired depth in meters
desired_depth_m = 2.0
# Approx: 1 m ≈ 10 hPa in seawater
pressure_offset = desired_depth_m * 10.0
fake_pressure = surface_pressure + pressure_offset

try:
    while True:
        master.mav.scaled_pressure_send(
            int(time.time() * 1000) % 65535,
            fake_pressure,    # press_abs
            0.0,             # press_diff (unused)
            2000             # temperature in cdegC
        )
        time.sleep(0.1)  # 10 Hz
except KeyboardInterrupt:
    print("Stopped")
