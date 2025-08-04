#!/usr/bin/env python3
"""
Autonomous AUV Dive Sequence
Run this script on the AUV's onboard computer (Raspberry Pi)
It will execute completely independently after starting
"""

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
        # Set mode to MANUAL
        mode_mapping = self.master.mode_mapping()
        mode_id = mode_mapping['MANUAL']
        self.master.set_mode(mode_id)
        print("[INFO] Set to MANUAL mode")
        time.sleep(2)
    
    def send_rc_command(self, channels, duration, description):
        """Send RC override for specified duration"""
        if not self.running:
            return False
            
        print(f"[ACTION] {description}")
        rc_override = [1500] * 8 + [65535] * 10
        
        # Apply channel values
        for channel, value in channels.items():
            rc_override[channel] = value
        
        start_time = time.time()
        while time.time() - start_time < duration and self.running:
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *rc_override
            )
            time.sleep(0.1)
        
        # Always stop movement after each command
        self.stop_movement()
        print(f"[COMPLETE] {description} - DONE")
        return self.running
    
    def stop_movement(self):
        """Stop all movement"""
        rc_override = [1500] * 8 + [65535] * 10
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *rc_override
        )
    
    def arm(self):
        """Arm the vehicle"""
        print("[1] Arming the vehicle...")
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        print("[SUCCESS] Vehicle armed")
        return True
    
    def disarm(self):
        """Disarm the vehicle"""
        print("[DISARM] Disarming the vehicle...")
        self.master.arducopter_disarm()
        self.master.motors_disarmed_wait()
        print("[SUCCESS] Vehicle disarmed")
    
    def execute_mission(self):
        """Execute the autonomous dive sequence"""
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
                return
            
            # 2. Dive for 2s
            if not self.send_rc_command({2: 1400}, 2, "[2] Diving for 2 seconds..."):
                return
            
            # 3. Stabilize for 3s
            print("[3] Stabilizing for 3 seconds...")
            time.sleep(3)
            if not self.running:
                return
            
            # 4. Move Forward for 2s
            if not self.send_rc_command({4: 1600, 5: 1600}, 2, "[4] Moving forward for 2 seconds..."):
                return
            
            # 5. Stabilize for 3s
            print("[5] Stabilizing for 3 seconds...")
            time.sleep(3)
            if not self.running:
                return
            
            # 6. Yaw 720 degrees right for 2s
            if not self.send_rc_command({4: 1600, 5: 1400}, 2, "[6] Yawing right for 2 seconds..."):
                return
            
            # 7. Come to surface for 2s
            if not self.send_rc_command({2: 1600}, 2, "[7] Coming to surface for 2 seconds..."):
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
        
        # Execute the mission
        auv.execute_mission()
        
    except KeyboardInterrupt:
        print("\n[INFO] Mission interrupted by user")
    except Exception as e:
        print(f"[ERROR] Critical failure: {e}")

if __name__ == "__main__":
    main()