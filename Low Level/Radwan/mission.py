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
        
        # Connect to Pixhawk
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
        if 'MANUAL' not in mode_mapping:
            raise Exception("[ERROR] MANUAL mode not available in mode mapping.")
        mode_id = mode_mapping['MANUAL']
        self.master.set_mode(mode_id)
        print("[INFO] Set to MANUAL mode.")
        time.sleep(1)
    
    def stop_movement(self):
        """Stop all movement"""
        rc_override = [1500] * 8 + [65535] * 10
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *rc_override
        )
        print("[INFO] Motion stopped.")
    
    def arm(self):
        """Arm the vehicle"""
        print("[INFO] Arming the vehicle...")
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        print("[SUCCESS] Vehicle armed.")
        return True
    
    def disarm(self):
        """Disarm the vehicle"""
        print("[INFO] Disarming the vehicle...")
        self.master.arducopter_disarm()
        self.master.motors_disarmed_wait()
        print("[SUCCESS] Vehicle disarmed.")
    
    def send_rc_movement(self, channel_values, duration, description):
        """Send RC override for specified duration"""
        if not self.running:
            return False
            
        print(f"[ACTION] {description}")
        
        # RC override array: 8 channels + 10 spares
        rc_override = [1500] * 8 + [65535] * 10
        
        # Apply channel values
        for channel, value in channel_values.items():
            rc_override[channel] = value
        
        start_time = time.time()
        while time.time() - start_time < duration and self.running:
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *rc_override
            )
            time.sleep(0.1)
        
        # Stop motion
        self.stop_movement()
        return self.running
    
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
            
            # 2. Dive for 2 seconds
            print("[ACTION] Diving for 2 seconds...")
            rc_override = [1500] * 8 + [65535] * 10
            rc_override[2] = 1400  # Dive down
            
            start_time = time.time()
            while time.time() - start_time < 2 and self.running:
                self.master.mav.rc_channels_override_send(
                    self.master.target_system,
                    self.master.target_component,
                    *rc_override
                )
                time.sleep(0.1)
            
            self.stop_movement()
            
            # 3. Move Forward for 5 seconds
            print("[ACTION] Moving forward for 5 seconds...")
            rc_override = [1500] * 8 + [65535] * 10
            rc_override[4] = 1600  # Forward thrust
            rc_override[5] = 1600  # Forward thrust
            
            start_time = time.time()
            while time.time() - start_time < 5 and self.running:
                self.master.mav.rc_channels_override_send(
                    self.master.target_system,
                    self.master.target_component,
                    *rc_override
                )
                time.sleep(0.1)
            
            self.stop_movement()
            
            # 4. Yaw 720 degrees right (differential thrust)
            print("[ACTION] Yawing 720 degrees right for 8 seconds...")
            rc_override = [1500] * 8 + [65535] * 10
            rc_override[4] = 1600  # One side forward
            rc_override[5] = 1400  # Other side reverse
            
            start_time = time.time()
            while time.time() - start_time < 8 and self.running:
                self.master.mav.rc_channels_override_send(
                    self.master.target_system,
                    self.master.target_component,
                    *rc_override
                )
                time.sleep(0.1)
            
            self.stop_movement()
            
            # 5. Stabilize for 2 seconds
            print("[INFO] Stabilizing for 2 seconds...")
            time.sleep(2)
            if not self.running:
                return
            
            # 6. Come to surface
            print("[ACTION] Coming to surface for 6 seconds...")
            rc_override = [1500] * 8 + [65535] * 10
            rc_override[2] = 1600  # Surface up
            
            start_time = time.time()
            while time.time() - start_time < 6 and self.running:
                self.master.mav.rc_channels_override_send(
                    self.master.target_system,
                    self.master.target_component,
                    *rc_override
                )
                time.sleep(0.1)
            
            self.stop_movement()
            
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