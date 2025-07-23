from pymavlink import mavutil

# Connect to the MAVLink stream
# If you're using BlueOS and QGroundControl, the vehicle likely broadcasts on this UDP port
connection = mavutil.mavlink_connection('udp:0.0.0.0:14550')

# Wait for the first heartbeat 
connection.wait_heartbeat()
print("Connected to system %u component %u" % (connection.target_system, connection.target_component))

print("Listening for messages...\n")

try:
    while True:
        msg = connection.recv_match(blocking=True)
        if not msg:
            continue

        msg_type = msg.get_type()

        # Filter and log relevant messages
        if msg_type in ['SERVO_OUTPUT_RAW', 'BATTERY_STATUS', 'SYS_STATUS', 'SYSTEM_TIME']:
            print(f"[{msg_type}] {msg.to_dict()}")
        
        # Uncomment to print all messages
        else:
            print(f"[{msg_type}] {msg.to_dict()}")
except KeyboardInterrupt:
    print("Logging stopped.")
