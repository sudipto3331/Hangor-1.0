from pymavlink import mavutil
import time

# Connect to Pixhawk
print("[INFO] Connecting to Pixhawk...")
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')  # Change to '/dev/ttyAMA0' for onboard
master.wait_heartbeat()
print(f"[INFO] Connected to system {master.target_system}, component {master.target_component}")

def upload_autonomous_mission():
    """Upload a waypoint mission for autonomous execution"""
    
    # Clear existing mission
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    
    # Wait for acknowledgment
    msg = master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    if msg and msg.type == 0:
        print("[INFO] Mission cleared successfully")
    
    # Define mission items (waypoints)
    mission_items = [
        # Item 0: HOME position (required)
        {
            'seq': 0,
            'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            'current': 1,  # This is the current waypoint
            'autocontinue': 1,
            'param1': 0,  # Hold time
            'param2': 0,  # Acceptance radius
            'param3': 0,  # Pass through waypoint
            'param4': 0,  # Yaw angle
            'x': 0,  # Latitude (relative)
            'y': 0,  # Longitude (relative)
            'z': 0,  # Altitude (relative)
            'mission_type': mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        },
        
        # Item 1: Move forward (5 meters)
        {
            'seq': 1,
            'frame': mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            'current': 0,
            'autocontinue': 1,
            'param1': 3.0,  # Hold for 3 seconds
            'param2': 1.0,  # Acceptance radius
            'param3': 0,
            'param4': 0,
            'x': 5.0,  # 5 meters forward (North in body frame)
            'y': 0.0,
            'z': 0.0,
            'mission_type': mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        },
        
        # Item 2: Move backward (return to start)
        {
            'seq': 2,
            'frame': mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            'current': 0,
            'autocontinue': 1,
            'param1': 3.0,
            'param2': 1.0,
            'param3': 0,
            'param4': 0,
            'x': 0.0,  # Back to start
            'y': 0.0,
            'z': 0.0,
            'mission_type': mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        },
        
        # Item 3: Move left (3 meters)
        {
            'seq': 3,
            'frame': mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            'current': 0,
            'autocontinue': 1,
            'param1': 3.0,
            'param2': 1.0,
            'param3': 0,
            'param4': 0,
            'x': 0.0,
            'y': -3.0,  # 3 meters left (West in body frame)
            'z': 0.0,
            'mission_type': mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        },
        
        # Item 4: Move right (return to center)
        {
            'seq': 4,
            'frame': mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            'current': 0,
            'autocontinue': 1,
            'param1': 3.0,
            'param2': 1.0,
            'param3': 0,
            'param4': 0,
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'mission_type': mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        },
        
        # Item 5: Yaw 720 degrees
        {
            'seq': 5,
            'frame': mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            'command': mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            'current': 0,
            'autocontinue': 1,
            'param1': 720,  # 720 degrees
            'param2': 90,   # 90 deg/sec yaw rate
            'param3': 1,    # Direction: 1=clockwise, -1=counter-clockwise
            'param4': 0,    # Relative angle
            'x': 0, 'y': 0, 'z': 0,
            'mission_type': mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        },
        
        # Item 6: Move up (2 meters)
        {
            'seq': 6,
            'frame': mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            'current': 0,
            'autocontinue': 1,
            'param1': 3.0,
            'param2': 1.0,
            'param3': 0,
            'param4': 0,
            'x': 0.0,
            'y': 0.0,
            'z': -2.0,  # Negative Z = up in NED frame
            'mission_type': mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        },
        
        # Item 7: Move down (return to original depth)  
        {
            'seq': 7,
            'frame': mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            'current': 0,
            'autocontinue': 1,
            'param1': 3.0,
            'param2': 1.0,
            'param3': 0,
            'param4': 0,
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'mission_type': mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        }
    ]
    
    # Send mission count
    master.mav.mission_count_send(
        master.target_system,
        master.target_component,
        len(mission_items),
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    )
    
    # Upload each mission item
    for item in mission_items:
        # Wait for mission request
        msg = master.recv_match(type='MISSION_REQUEST', blocking=True, timeout=10)
        if not msg:
            print(f"[ERROR] Timeout waiting for mission request for item {item['seq']}")
            return False
        
        # Send mission item
        master.mav.mission_item_send(
            master.target_system,
            master.target_component,
            item['seq'],
            item['frame'],
            item['command'],
            item['current'],
            item['autocontinue'],
            item['param1'],
            item['param2'],
            item['param3'],
            item['param4'],
            item['x'],
            item['y'],
            item['z'],
            item['mission_type']
        )
        print(f"[INFO] Uploaded mission item {item['seq']}")
    
    # Wait for mission acknowledgment
    msg = master.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
    if msg and msg.type == 0:
        print("[SUCCESS] Mission uploaded successfully!")
        return True
    else:
        print("[ERROR] Mission upload failed")
        return False

def start_autonomous_mission():
    """Start the autonomous mission"""
    
    # Set mode to AUTO
    mode_mapping = master.mode_mapping()
    if 'AUTO' not in mode_mapping:
        print("[ERROR] AUTO mode not available")
        return False
    
    mode_id = mode_mapping['AUTO']
    master.set_mode(mode_id)
    print("[INFO] Set to AUTO mode")
    time.sleep(2)
    
    # Arm the vehicle
    print("[INFO] Arming vehicle...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("[SUCCESS] Vehicle armed")
    
    # Start mission
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0,  # Confirmation
        0, 0, 0, 0, 0, 0, 0  # Parameters (unused for this command)
    )
    print("[INFO] Mission started!")
    
    return True

def monitor_mission():
    """Monitor mission progress"""
    print("[INFO] Monitoring mission progress...")
    
    while True:
        # Get mission current item
        msg = master.recv_match(type='MISSION_CURRENT', blocking=False)
        if msg:
            print(f"[STATUS] Current mission item: {msg.seq}")
        
        # Check if mission is complete
        msg = master.recv_match(type='MISSION_ITEM_REACHED', blocking=False)
        if msg:
            print(f"[STATUS] Reached waypoint {msg.seq}")
        
        # Check for mission completion or errors
        msg = master.recv_match(type='STATUSTEXT', blocking=False)
        if msg:
            text = msg.text.decode('utf-8').strip()
            if 'mission' in text.lower():
                print(f"[MISSION] {text}")
                if 'complete' in text.lower():
                    break
        
        time.sleep(1)

# Main execution
if __name__ == "__main__":
    try:
        print("=== Autonomous AUV Mission ===")
        
        # Upload the mission
        if upload_autonomous_mission():
            # Start the mission
            if start_autonomous_mission():
                # Monitor until completion
                monitor_mission()
            else:
                print("[ERROR] Failed to start mission")
        else:
            print("[ERROR] Failed to upload mission")
        
        # Disarm when done
        print("[INFO] Disarming vehicle...")
        master.arducopter_disarm()
        master.motors_disarmed_wait()
        print("[SUCCESS] Mission complete!")
        
    except KeyboardInterrupt:
        print("\n[INFO] Mission interrupted by user")
        master.arducopter_disarm()
    except Exception as e:
        print(f"[ERROR] Mission failed: {e}")
        master.arducopter_disarm()