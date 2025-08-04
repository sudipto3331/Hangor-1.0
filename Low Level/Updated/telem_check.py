from pymavlink import mavutil
import time

# Connect to Pixhawk via USB (adjust if using UART or UDP)
print("[INFO] Connecting to Pixhawk via USB...")
#master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# Wait for the heartbeat
print("[INFO] Waiting for heartbeat from Pixhawk...")
master.wait_heartbeat()
print(f"[INFO] Heartbeat received from system (system {master.target_system}, component {master.target_component})")

print("[INFO] Listening for RAW_IMU or SCALED_IMU messages for gyroscope data...")

try:
    while True:
        msg = master.recv_match(type=["RAW_IMU", "SCALED_IMU"], blocking=True)
        if msg:
            # Gyroscope values (in milli-degrees/sec or deg/sec)
            if msg.get_type() == "RAW_IMU":
                gx = msg.xgyro
                gy = msg.ygyro
                gz = msg.zgyro
                print(f"[RAW_IMU] Gyro (x, y, z): {gx}, {gy}, {gz}")
            elif msg.get_type() == "SCALED_IMU":
                gx = msg.xgyro / 1000.0
                gy = msg.ygyro / 1000.0
                gz = msg.zgyro / 1000.0
                print(f"[SCALED_IMU] Gyro (x, y, z): {gx:.3f}, {gy:.3f}, {gz:.3f}")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n[INFO] Script interrupted by user. Exiting...")
