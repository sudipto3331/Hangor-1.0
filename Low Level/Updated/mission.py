#!/usr/bin/env python3
"""
Autonomous ROV Mission Script
Performs: Arm -> Dive 2s -> Forward 5s -> Yaw 720° -> Surface -> Disarm
Runs independently even if ethernet connection is lost.
"""

from pymavlink import mavutil
import time
import sys
import signal
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('rov_mission.log'),
        logging.StreamHandler(sys.stdout)
    ]
)

class ROVMission:
    def __init__(self, connection_string='udpin:0.0.0.0:14550'):
        self.master = None
        self.connection_string = connection_string
        self.mission_active = True
        
        # RC override array: 8 channels + 10 spares
        self.rc_override = [1500] * 8 + [65535] * 10
        
        # Configure signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        logging.info(f"[SHUTDOWN] Received signal {signum}, shutting down gracefully...")
        self.mission_active = False
        if self.master:
            self.emergency_stop()
    
    def connect_to_pixhawk(self, max_retries=10, retry_delay=5):
        """Connect to Pixhawk with retry logic"""
        for attempt in range(max_retries):
            try:
                logging.info(f"[CONNECTION] Attempting to connect to Pixhawk (attempt {attempt + 1}/{max_retries})...")
                self.master = mavutil.mavlink_connection(self.connection_string)
                self.master.wait_heartbeat(timeout=10)
                logging.info(f"[SUCCESS] Connected to system {self.master.target_system}, component {self.master.target_component}")
                return True
            except Exception as e:
                logging.error(f"[ERROR] Connection attempt {attempt + 1} failed: {e}")
                if attempt < max_retries - 1:
                    logging.info(f"[RETRY] Retrying in {retry_delay} seconds...")
                    time.sleep(retry_delay)
        
        logging.error("[FATAL] Failed to connect to Pixhawk after all attempts")
        return False
    
    def set_mode(self, mode_name='MANUAL'):
        """Set vehicle mode"""
        try:
            mode_mapping = self.master.mode_mapping()
            if mode_name not in mode_mapping:
                raise Exception(f"[ERROR] {mode_name} mode not available in mode mapping.")
            
            mode_id = mode_mapping[mode_name]
            self.master.set_mode(mode_id)
            logging.info(f"[MODE] Set to {mode_name} mode.")
            time.sleep(1)
            return True
        except Exception as e:
            logging.error(f"[ERROR] Failed to set mode: {e}")
            return False
    
    def arm_vehicle(self):
        """Arm the vehicle"""
        try:
            logging.info("[ARM] Arming the vehicle...")
            self.master.arducopter_arm()
            self.master.motors_armed_wait(timeout=10)
            logging.info("[SUCCESS] Vehicle armed.")
            return True
        except Exception as e:
            logging.error(f"[ERROR] Failed to arm vehicle: {e}")
            return False
    
    def disarm_vehicle(self):
        """Disarm the vehicle"""
        try:
            logging.info("[DISARM] Disarming the vehicle...")
            # Stop all motion first
            self.stop_all_motion()
            self.master.arducopter_disarm()
            self.master.motors_disarmed_wait(timeout=10)
            logging.info("[SUCCESS] Vehicle disarmed.")
            return True
        except Exception as e:
            logging.error(f"[ERROR] Failed to disarm vehicle: {e}")
            return False
    
    def send_rc_override(self):
        """Send RC override command"""
        try:
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *self.rc_override
            )
        except Exception as e:
            logging.error(f"[ERROR] Failed to send RC override: {e}")
    
    def stop_all_motion(self):
        """Stop all motion by setting all channels to neutral"""
        self.rc_override = [1500] * 8 + [65535] * 10
        self.send_rc_override()
        logging.info("[STOP] All motion stopped.")
    
    def dive(self, duration=2):
        """Dive for specified duration"""
        logging.info(f"[DIVE] Diving for {duration} seconds...")
        
        # Set vertical thruster for diving (assuming channel 3 for vertical)
        self.rc_override[2] = 1400  # Dive down
        
        start_time = time.time()
        while time.time() - start_time < duration and self.mission_active:
            self.send_rc_override()
            time.sleep(0.1)
        
        # Stop vertical motion
        self.rc_override[2] = 1500
        self.send_rc_override()
        logging.info("[DIVE] Dive complete.")
    
    def move_forward(self, duration=5):
        """Move forward for specified duration"""
        logging.info(f"[FORWARD] Moving forward for {duration} seconds...")
        
        # Set forward thrusters (assuming channels 5 and 6 for forward motion)
        self.rc_override[4] = 1600  # Forward thrust
        self.rc_override[5] = 1600
        
        start_time = time.time()
        while time.time() - start_time < duration and self.mission_active:
            self.send_rc_override()
            time.sleep(0.1)
        
        # Stop forward motion
        self.rc_override[4] = 1500
        self.rc_override[5] = 1500
        self.send_rc_override()
        logging.info("[FORWARD] Forward motion complete.")
    
    def yaw_rotation(self, degrees=720, rotation_speed=90):
        """Perform yaw rotation"""
        logging.info(f"[YAW] Performing {degrees}° yaw rotation...")
        
        # Calculate duration based on rotation speed (degrees per second)
        duration = abs(degrees) / rotation_speed
        
        # Set yaw thruster (assuming channel 4 for yaw)
        if degrees > 0:
            self.rc_override[3] = 1600  # Clockwise
        else:
            self.rc_override[3] = 1400  # Counter-clockwise
        
        start_time = time.time()
        while time.time() - start_time < duration and self.mission_active:
            self.send_rc_override()
            time.sleep(0.1)
        
        # Stop yaw motion
        self.rc_override[3] = 1500
        self.send_rc_override()
        logging.info("[YAW] Yaw rotation complete.")
    
    def surface(self, duration=3):
        """Surface for specified duration"""
        logging.info(f"[SURFACE] Surfacing for {duration} seconds...")
        
        # Set vertical thruster for surfacing
        self.rc_override[2] = 1600  # Surface up
        
        start_time = time.time()
        while time.time() - start_time < duration and self.mission_active:
            self.send_rc_override()
            time.sleep(0.1)
        
        # Stop vertical motion
        self.rc_override[2] = 1500
        self.send_rc_override()
        logging.info("[SURFACE] Surfacing complete.")
    
    def emergency_stop(self):
        """Emergency stop procedure"""
        logging.warning("[EMERGENCY] Performing emergency stop...")
        self.stop_all_motion()
        time.sleep(0.5)
        self.disarm_vehicle()
    
    def run_mission(self):
        """Execute the complete autonomous mission"""
        logging.info("[MISSION] Starting autonomous ROV mission...")
        
        try:
            # Step 1: Connect to Pixhawk
            if not self.connect_to_pixhawk():
                return False
            
            # Step 2: Set mode to MANUAL
            if not self.set_mode('MANUAL'):
                return False
            
            # Step 3: Arm the vehicle
            if not self.arm_vehicle():
                return False
            
            # Wait a moment after arming
            time.sleep(2)
            
            # Step 4: Dive for 2 seconds
            if self.mission_active:
                self.dive(2)
            
            # Wait between maneuvers
            if self.mission_active:
                time.sleep(1)
            
            # Step 5: Move forward for 5 seconds
            if self.mission_active:
                self.move_forward(5)
            
            # Wait between maneuvers
            if self.mission_active:
                time.sleep(1)
            
            # Step 6: Yaw 720 degrees
            if self.mission_active:
                self.yaw_rotation(720)
            
            # Wait between maneuvers
            if self.mission_active:
                time.sleep(1)
            
            # Step 7: Surface
            if self.mission_active:
                self.surface(3)
            
            # Wait before disarming
            if self.mission_active:
                time.sleep(2)
            
            # Step 8: Disarm the vehicle
            self.disarm_vehicle()
            
            logging.info("[SUCCESS] Mission completed successfully!")
            return True
            
        except Exception as e:
            logging.error(f"[FATAL] Mission failed with error: {e}")
            self.emergency_stop()
            return False
        
        finally:
            if self.master:
                self.stop_all_motion()

def main():
    """Main function to run the autonomous mission"""
    logging.info("="*50)
    logging.info("AUTONOMOUS ROV MISSION STARTING")
    logging.info("="*50)
    
    # Create mission instance
    mission = ROVMission()
    
    # Run the mission
    success = mission.run_mission()
    
    if success:
        logging.info("[COMPLETE] Mission executed successfully!")
        sys.exit(0)
    else:
        logging.error("[FAILED] Mission failed!")
        sys.exit(1)

if __name__ == "__main__":
    main()