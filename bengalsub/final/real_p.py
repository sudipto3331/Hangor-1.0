from pymavlink import mavutil

# Connect to Pixhawk - change IP/port if needed
# If using BlueOS MAVLink Router, create a new endpoint like 'udpin:0.0.0.0:14560'
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# Wait for heartbeat to confirm connection
master.wait_heartbeat()
print("Connected to Pixhawk")

try:
    while True:
        # Wait for the next SCALED_PRESSURE message
        msg = master.recv_match(type='SCALED_PRESSURE', blocking=True)
        if msg:
            # Pressure in hPa, temperature in cdegC
            print(f"Pressure: {msg.press_abs:.2f} hPa, Temp: {msg.temperature/100:.2f} °C")

except KeyboardInterrupt:
    print("\nStopped by user")
