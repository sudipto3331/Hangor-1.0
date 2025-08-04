from pymavlink import mavutil
import time
import signal
import sys

class AutonomousAUV:
    def __init__(self):
        self.running = True
        # Setup graceful shutdown
        signal.signal(signal.SIGINT, self.shutdown)
        signal.signal(signal.SIGTERM, self.shutdown)
        
        # Connect via serial (onboard connection)
        print("[INFO] Connecting to Pixhawk...")
        self.master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
        self.master.wait_heartbeat()
        print(f"[INFO] Connected to system {self.master.target_system}, component {self.master.target_component}")
    
    def shutdown(self, signum, frame):
        """Graceful shutdown handler"""
        print("\n[SHUTDOWN] Emergency stop initiated...")
        self.running = False
        self.stop_movement()
        self.disarm()
        sys.exit(0)
    
    def setup(self):
        """Setup AUV for autonomous operation"""
        # Set mode to MANUAL for direct thruster control
        mode_mapping = self.master.mode_mapping()
        if 'MANUAL' in mode_mapping:
            mode_id = mode_mapping['MANUAL']
        else:
            # Fallback to STABILIZE if MANUAL not available
            mode_id = mode_mapping.get('STABILIZE', 0)
        
        self.master.set_mode(mode_id)
        print(f"[INFO] Set to mode ID: {mode_id}")
        time.sleep(2)
        
        # Send parameter request to ensure connection is stable
        self.master.mav.param_request_list_send(
            self.master.target_system,
            self.master.target_component
        )
    
    def send_rc_command(self, channels, duration, description):
        """Send RC override for specified duration with debugging"""
        if not self.running:
            return False
            
        print(f"[ACTION] {description}")
        print(f"[DEBUG] Channel commands: {channels}")
        
        # Initialize all channels to neutral (1500)
        rc_override = [1500] * 8 + [65535] * 10
        
        # Apply channel values
        for channel, value in channels.items():
            if 0 <= channel < 8:  # Ensure valid channel range
                rc_override[channel] = value
                print(f"[DEBUG] Setting channel {channel+1} to {value}")
        
        start_time = time.time()
        command_count = 0
        
        while time.time() - start_time < duration and self.running:
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *rc_override
            )
            command_count += 1
            time.sleep(0.1)
        
        print(f"[DEBUG] Sent {command_count} RC commands over {duration}s")
        self.stop_movement()
        return self.running
    
    def stop_movement(self):
        """Stop all movement"""
        print("[DEBUG] Stopping all movement...")
        rc_override = [1500] * 8 + [65535] * 10
        # Send stop command multiple times to ensure it's received
        for _ in range(5):
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *rc_override
            )
            time.sleep(0.05)
    
    def test_individual_thrusters(self):
        """Test each thruster individually for debugging"""
        print("\n=== THRUSTER TEST SEQUENCE ===")
        
        # Test common thruster channel mappings
        test_channels = [
            (0, "Thruster 1 (usually forward/back)"),
            (1, "Thruster 2 (usually up/down)"), 
            (2, "Thruster 3 (usually left/right)"),
            (3, "Thruster 4 (usually yaw)"),
            (4, "Thruster 5"),
            (5, "Thruster 6")
        ]
        
        for channel, description in test_channels:
            if not self.running:
                break
                
            print(f"\n[TEST] Testing {description}")
            print("Press Enter to continue or Ctrl+C to skip...")
            try:
                input()
            except KeyboardInterrupt:
                break
                
            # Test forward direction
            self.send_rc_command({channel: 1600}, 2, f"Testing channel {channel+1} forward")
            time.sleep(1)
            
            # Test reverse direction  
            self.send_rc_command({channel: 1400}, 2, f"Testing channel {channel+1} reverse")
            time.sleep(1)
    
    def arm(self):
        """Arm the vehicle with better error handling"""
        print("[1] Arming the vehicle...")
        
        # Check if already armed
        self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        heartbeat = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if heartbeat and heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("[INFO] Vehicle already armed")
            return True
        
        # Send arm command
        self.master.arducopter_arm()
        
        # Wait for arming with timeout
        start_time = time.time()
        while time.time() - start_time < 10:  # 10 second timeout
            heartbeat = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if heartbeat and heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print("[SUCCESS] Vehicle armed")
                return True
            time.sleep(0.5)
        
        print("[ERROR] Failed to arm vehicle within 10 seconds")
        return False
    
    def disarm(self):
        """Disarm the vehicle"""
        print("[DISARM] Disarming the vehicle...")
        self.master.arducopter_disarm()
        
        # Wait for disarming
        start_time = time.time()
        while time.time() - start_time < 5:
            heartbeat = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if heartbeat and not (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("[SUCCESS] Vehicle disarmed")
                return
            time.sleep(0.5)
        
        print("[WARNING] Disarm status unclear")
    
    def execute_mission(self):
        """Execute the autonomous dive sequence with corrected channel mapping"""
        print("\n=== AUTONOMOUS DIVE SEQUENCE STARTING ===")
        
        # Safety countdown
        for i in range(5, 0, -1):
            if not self.running:
                return
            print(f"[COUNTDOWN] Starting in {i}...")
            time.sleep(1)
        
        try:
            # 1. Arm the vehicle
            if not self.arm():
                print("[ERROR] Failed to arm - aborting mission")
                return
            
            # 2. Dive 1m (Channel 2 is typically throttle/up-down in ArduSub)
            # Using 0-based indexing: Channel 2 = rc_override[2] = RC channel 3
            if not self.send_rc_command({2: 1400}, 5, "[2] Diving 1 meter..."):
                return
            
            # 3. Stabilize for 3s
            print("[3] Stabilizing for 3 seconds...")
            time.sleep(3)
            if not self.running:
                return
            
            # 4. Move Forward 4m
            # Channel 0 = forward/back (pitch), Channel 1 = left/right (roll) 
            # Using both for forward motion - adjust based on your frame configuration
            if not self.send_rc_command({0: 1600}, 6, "[4] Moving forward 4 meters..."):
                return
            
            # 5. Stabilize for 3s
            print("[5] Stabilizing for 3 seconds...")
            time.sleep(3)
            if not self.running:
                return
            
            # 6. Yaw 720 degrees right
            # Channel 3 is typically yaw
            if not self.send_rc_command({3: 1600}, 8, "[6] Yawing 720 degrees right..."):
                return
            
            # 7. Come to surface
            if not self.send_rc_command({2: 1600}, 6, "[7] Coming to surface..."):
                return
            
            print("\n[SUCCESS] Mission completed successfully!")
            
        except Exception as e:
            print(f"[ERROR] Mission failed: {e}")
        finally:
            # 8. Always disarm at the end
            self.disarm()

def main():
    """Main execution function"""
    print("=== AUTONOMOUS AUV DIVE SEQUENCE ===")
    print("This script will run independently on the AUV")
    print("Press Ctrl+C to emergency stop\n")
    
    try:
        # Create autonomous AUV instance
        auv = AutonomousAUV()
        
        # Setup the vehicle
        auv.setup()
        
        # Ask user if they want to test thrusters first
        print("\nWould you like to test individual thrusters first? (y/n)")
        try:
            response = input().lower()
            if response == 'y':
                auv.test_individual_thrusters()
                print("\nProceed with mission? (y/n)")
                response = input().lower()
                if response != 'y':
                    print("Mission cancelled by user")
                    return
        except KeyboardInterrupt:
            print("\nSkipping thruster test...")
        
        # Execute the mission
        auv.execute_mission()
        
    except KeyboardInterrupt:
        print("\n[INFO] Mission interrupted by user")
    except Exception as e:
        print(f"[ERROR] Critical failure: {e}")

if __name__ == "__main__":
    main()