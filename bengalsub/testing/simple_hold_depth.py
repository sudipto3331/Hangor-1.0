import time
from pymavlink import mavutil
# Remove incorrect import: from doctest import master

# Configuration
TARGET_DEPTH = 0.3048  # 1 foot in meters (ArduSub uses meters)
STABILIZE_TIME = 10    # seconds
DEPTH_TOLERANCE = 0.05 # 5cm tolerance for depth holding

def wait_heartbeat(master):
    """Wait for a heartbeat to ensure connection"""
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")

def arm_vehicle(master):
    """Arm the vehicle"""
    print("Arming vehicle...")
    master.arducopter_arm()
    
    # Wait for arming confirmation
    master.motors_armed_wait()
    print("Vehicle armed!")

def disarm_vehicle(master):
    """Disarm the vehicle"""
    print("Disarming vehicle...")
    master.arducopter_disarm()
    
    # Wait for disarming confirmation
    master.motors_disarmed_wait()
    print("Vehicle disarmed!")

def set_mode(master, mode):
    """Set flight mode"""
    if mode not in master.mode_mapping():
        print(f"Unknown mode: {mode}")
        return False
    
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    
    # Wait for mode change confirmation
    print(f"Setting mode to {mode}...")
    while True:
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack_msg:
            if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"Mode changed to {mode}")
                return True
            else:
                print(f"Mode change failed: {ack_msg.result}")
                return False
        
        # Check current mode via heartbeat
        heartbeat = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if heartbeat and heartbeat.custom_mode == mode_id:
            print(f"Mode changed to {mode}")
            return True

def get_depth_m(master):
    """Get current depth from Bar30 pressure sensor"""
    msg = master.recv_match(type=['SCALED_PRESSURE', 'SCALED_PRESSURE2'], blocking=True, timeout=1)
    if not msg:
        return None
    # Convert pressure to depth:
    # Assuming msg.press_abs in mbar, and water density ~ 1000 kg/m³
    pressure_mbar = msg.press_abs
    pressure_pa = pressure_mbar * 100  # mbar → Pa
    depth = (pressure_pa - 101325) / (1000 * 9.80665)
    return max(depth, 0)

def dive_to_depth(master, target_depth):
    """Dive to target depth using rc_channels override"""
    print(f"Diving to {target_depth}m depth...")
    
    # Set ALT_HOLD mode for depth control
    if not set_mode(master, 'ALT_HOLD'):
        print("Failed to set ALT_HOLD mode")
        return False
    
    time.sleep(2)  # Allow mode to stabilize
    
    # Use RC channel override for diving (channel 3 is typically throttle/vertical)
    # Values: 1500 = neutral, <1500 = down, >1500 = up
    dive_pwm = 1400  # Adjust this value as needed for your setup
    
    start_time = time.time()
    max_dive_time = 30  # Maximum time to attempt diving (safety)
    
    while time.time() - start_time < max_dive_time:
        current_depth = get_depth_m(master)
        
        if current_depth is not None:
            print(f"Current depth: {current_depth:.3f}m, Target: {target_depth:.3f}m")
            
            if abs(current_depth - target_depth) < DEPTH_TOLERANCE:
                print("Target depth reached!")
                # Send neutral throttle to stop diving
                master.mav.rc_channels_override_send(
                    master.target_system,
                    master.target_component,
                    0, 0, 1500, 0, 0, 0, 0, 0  # Only channel 3 (throttle) set to neutral
                )
                return True
            elif current_depth < target_depth:
                # Need to dive deeper
                master.mav.rc_channels_override_send(
                    master.target_system,
                    master.target_component,
                    0, 0, dive_pwm, 0, 0, 0, 0, 0
                )
            else:
                # Too deep, ascend slightly
                master.mav.rc_channels_override_send(
                    master.target_system,
                    master.target_component,
                    0, 0, 1600, 0, 0, 0, 0, 0
                )
        
        time.sleep(0.5)  # Check depth every 500ms
    
    print("Dive timeout reached")
    return False

def stabilize_depth(master, target_depth, duration):
    """Maintain depth for specified duration"""
    print(f"Stabilizing at {target_depth}m for {duration} seconds...")
    
    start_time = time.time()
    
    while time.time() - start_time < duration:
        current_depth = get_depth_m(master)
        
        if current_depth is not None:
            depth_error = current_depth - target_depth
            print(f"Depth: {current_depth:.3f}m, Error: {depth_error:.3f}m, Time remaining: {duration - (time.time() - start_time):.1f}s")
            
            # Simple depth control
            if abs(depth_error) > DEPTH_TOLERANCE:
                if depth_error > 0:  # Too deep
                    pwm = 1550
                else:  # Too shallow
                    pwm = 1450
                
                master.mav.rc_channels_override_send(
                    master.target_system,
                    master.target_component,
                    0, 0, pwm, 0, 0, 0, 0, 0
                )
            else:
                # Maintain neutral
                master.mav.rc_channels_override_send(
                    master.target_system,
                    master.target_component,
                    0, 0, 1500, 0, 0, 0, 0, 0
                )
        
        time.sleep(0.5)
    
    print("Stabilization complete!")

def clear_rc_override(master):
    """Clear RC channel overrides"""
    print("Clearing RC overrides...")
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, 0, 0, 0, 0, 0, 0  # All channels to 0 (no override)
    )

def main():
    """Main execution function"""
    # Create MAVLink connection
    print("Connecting to ArduSub...")
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    
    try:
        # Wait for heartbeat
        wait_heartbeat(master)
        
        # Arm vehicle
        arm_vehicle(master)
        
        # Dive to target depth
        if dive_to_depth(master, TARGET_DEPTH):
            # Stabilize at depth
            stabilize_depth(master, TARGET_DEPTH, STABILIZE_TIME)
        else:
            print("Failed to reach target depth")
        
        # Clear RC overrides
        clear_rc_override(master)
        
        # Switch to MANUAL mode before disarming
        set_mode(master, 'MANUAL')
        time.sleep(2)
        
        # Disarm vehicle
        disarm_vehicle(master)
        
        print("Mission complete!")
        
    except KeyboardInterrupt:
        print("\nMission interrupted by user")
        clear_rc_override(master)
        disarm_vehicle(master)
    except Exception as e:
        print(f"Error occurred: {e}")
        clear_rc_override(master)
        disarm_vehicle(master)
    finally:
        master.close()

if __name__ == "__main__":
    main()