"""
Mission executor for ROBOSUB25 AUV
Executes the complete mission sequence
"""

import asyncio
import logging
import time
import math
from auv.control.pid_controller import AUVPIDControllers
from auv.vision.utils.alignment import AlignmentController
from config.settings import AUV_CONFIG

class MissionState:
    INITIALIZING = "initializing"
    DIVING = "diving"
    STABILIZING = "stabilizing"
    DETECTING = "detecting"
    ALIGNING = "aligning"
    PASSING_GATE = "passing_gate"
    ROTATING = "rotating"
    SURFACING = "surfacing"
    COMPLETED = "completed"
    ABORTED = "aborted"

class MissionExecutor:
    def __init__(self, sensor_manager, thruster_controller, object_detector):
        self.logger = logging.getLogger("MissionExecutor")
        
        # System components
        self.sensor_manager = sensor_manager
        self.thruster_controller = thruster_controller
        self.object_detector = object_detector
        
        # Control systems
        self.pid_controllers = AUVPIDControllers()
        self.alignment_controller = AlignmentController(
            *AUV_CONFIG["SENSORS"]["OAK_D_RESOLUTION"]
        )
        
        # Mission state
        self.current_state = MissionState.INITIALIZING
        self.mission_start_time = None
        self.is_running = False
        
        # Mission parameters
        self.target_depth = AUV_CONFIG["MISSION"]["TARGET_DEPTH"]
        self.stabilize_time = AUV_CONFIG["MISSION"]["STABILIZE_TIME"]
        self.yaw_rotation = AUV_CONFIG["MISSION"]["YAW_ROTATION"]
        
    async def execute_complete_mission(self):
        """Execute the complete mission sequence"""
        self.logger.info("Starting mission execution...")
        self.mission_start_time = time.time()
        self.is_running = True
        
        try:
            # Mission sequence
            await self._mission_dive_to_depth()
            await self._mission_stabilize()
            await self._mission_detect_and_align()
            await self._mission_pass_through_gate()
            await self._mission_stabilize()
            await self._mission_perform_yaw_rotation()
            await self._mission_surface()
            
            self.current_state = MissionState.COMPLETED
            self.logger.info("Mission completed successfully!")
            
        except Exception as e:
            self.logger.error(f"Mission failed: {e}")
            self.current_state = MissionState.ABORTED
            await self.thruster_controller.stop_all_thrusters()
        
        self.is_running = False
    
    async def _mission_dive_to_depth(self):
        """Mission Step 1: Dive to target depth (1.5m)"""
        self.current_state = MissionState.DIVING
        self.logger.info(f"Diving to target depth: {self.target_depth}m")
        
        # Reset PID controllers
        self.pid_controllers.reset_all_controllers()
        
        start_time = time.time()
        depth_reached = False
        
        while not depth_reached and (time.time() - start_time) < 60:  # 60 second timeout
            sensor_data = self.sensor_manager.get_sensor_data()
            current_depth = sensor_data.depth
            
            # Calculate depth control output
            depth_output = self.pid_controllers.update_depth_control(
                current_depth, self.target_depth
            )
            
            # Calculate stabilization outputs for pitch and roll
            pitch_output = self.pid_controllers.update_pitch_control(sensor_data.pitch)
            roll_output = self.pid_controllers.update_roll_control(sensor_data.roll)
            
            # Apply vertical thrust for depth control
            vertical_thrust = [
                depth_output + pitch_output + roll_output,  # Front-left
                depth_output + pitch_output - roll_output,  # Front-right
                depth_output - pitch_output + roll_output,  # Back-left
                depth_output - pitch_output - roll_output   # Back-right
            ]
            
            await self.thruster_controller.set_vertical_thrust(vertical_thrust)
            
            # Check if depth is reached (within 0.1m tolerance)
            if abs(current_depth - self.target_depth) <= 0.1:
                depth_reached = True
            
            self.logger.info(f"Current depth: {current_depth:.2f}m, Target: {self.target_depth}m")
            await asyncio.sleep(0.1)
        
        if not depth_reached:
            raise Exception("Failed to reach target depth within timeout")
        
        self.logger.info(f"Target depth reached: {sensor_data.depth:.2f}m")
    
    async def _mission_stabilize(self):
        """Stabilize AUV for specified time"""
        self.current_state = MissionState.STABILIZING
        self.logger.info(f"Stabilizing for {self.stabilize_time} seconds")
        
        start_time = time.time()
        
        while (time.time() - start_time) < self.stabilize_time:
            sensor_data = self.sensor_manager.get_sensor_data()
            
            # Maintain depth
            depth_output = self.pid_controllers.update_depth_control(
                sensor_data.depth, self.target_depth
            )
            
            # Stabilize orientation
            pitch_output = self.pid_controllers.update_pitch_control(sensor_data.pitch)
            roll_output = self.pid_controllers.update_roll_control(sensor_data.roll)
            
            # Apply stabilization thrust
            vertical_thrust = [
                depth_output + pitch_output + roll_output,
                depth_output + pitch_output - roll_output,
                depth_output - pitch_output + roll_output,
                depth_output - pitch_output - roll_output
            ]
            
            await self.thruster_controller.set_vertical_thrust(vertical_thrust)
            await asyncio.sleep(0.1)
        
        # Stop horizontal movement
        await self.thruster_controller.set_horizontal_thrust([0, 0, 0, 0])
        self.logger.info("Stabilization complete")
    
    async def _mission_detect_and_align(self):
        """Mission Step 3: Detect fish and align with best detection"""
        self.current_state = MissionState.DETECTING
        self.logger.info("Starting fish detection and alignment")
        
        detection_timeout = 30  # 30 seconds to find a fish
        start_time = time.time()
        best_detection = None
        
        while (time.time() - start_time) < detection_timeout:
            # Get camera frame
            frame = self.sensor_manager.get_camera_frame()
            if frame is None:
                await asyncio.sleep(0.1)
                continue
            
            # Detect objects
            detections = await self.object_detector.detect_objects(frame)
            
            if detections:
                # Get best detection (highest confidence)
                best_detection = self.object_detector.get_best_detection(detections)
                self.logger.info(f"Detected {best_detection.class_name} with confidence {best_detection.confidence:.2f}")
                break
            
            # Maintain depth while searching
            sensor_data = self.sensor_manager.get_sensor_data()
            depth_output = self.pid_controllers.update_depth_control(
                sensor_data.depth, self.target_depth
            )
            
            vertical_thrust = [depth_output] * 4
            await self.thruster_controller.set_vertical_thrust(vertical_thrust)
            
            await asyncio.sleep(0.1)
        
        if not best_detection:
            raise Exception("No fish detected within timeout")
        
        # Align with detected fish
        self.current_state = MissionState.ALIGNING
        self.logger.info("Aligning with detected fish")
        
        alignment_timeout = 20  # 20 seconds to align
        start_time = time.time()
        
        while (time.time() - start_time) < alignment_timeout:
            frame = self.sensor_manager.get_camera_frame()
            if frame is None:
                await asyncio.sleep(0.1)
                continue
            
            # Detect objects in current frame
            detections = await self.object_detector.detect_objects(frame)
            current_detection = self.object_detector.get_best_detection(detections)
            
            if not current_detection:
                # Lost detection, try to find it again
                await asyncio.sleep(0.1)
                continue
            
            # Check if aligned
            if self.alignment_controller.is_aligned(current_detection):
                self.logger.info("Alignment complete!")
                break
            
            # Get alignment commands
            align_cmds = self.alignment_controller.get_alignment_commands(current_detection)
            
            # Maintain depth
            sensor_data = self.sensor_manager.get_sensor_data()
            depth_output = self.pid_controllers.update_depth_control(
                sensor_data.depth, self.target_depth
            )
            
            # Apply alignment movements
            horizontal_thrust = [
                align_cmds["forward"] + align_cmds["horizontal"],  # Front-left
                align_cmds["forward"] - align_cmds["horizontal"],  # Front-right
                align_cmds["forward"] + align_cmds["horizontal"],  # Back-left
                align_cmds["forward"] - align_cmds["horizontal"]   # Back-right
            ]
            
            vertical_thrust = [depth_output + align_cmds["vertical"]] * 4
            
            await self.thruster_controller.set_horizontal_thrust(horizontal_thrust)
            await self.thruster_controller.set_vertical_thrust(vertical_thrust)
            
            await asyncio.sleep(0.1)
    
    async def _mission_pass_through_gate(self):
        """Mission Step 4: Pass under the detected fish (gate)"""
        self.current_state = MissionState.PASSING_GATE
        self.logger.info("Passing through gate (under fish)")
        
        # Move forward for a fixed time to pass under the fish
        pass_duration = 5.0  # 5 seconds to pass through
        start_time = time.time()
        
        while (time.time() - start_time) < pass_duration:
            sensor_data = self.sensor_manager.get_sensor_data()
            
            # Maintain depth
            depth_output = self.pid_controllers.update_depth_control(
                sensor_data.depth, self.target_depth
            )
            
            # Move forward
            forward_thrust = 0.6
            horizontal_thrust = [forward_thrust] * 4
            vertical_thrust = [depth_output] * 4
            
            await self.thruster_controller.set_horizontal_thrust(horizontal_thrust)
            await self.thruster_controller.set_vertical_thrust(vertical_thrust)
            
            await asyncio.sleep(0.1)
        
        # Stop forward movement
        await self.thruster_controller.set_horizontal_thrust([0, 0, 0, 0])
        self.logger.info("Gate passage complete")
    
    async def _mission_perform_yaw_rotation(self):
        """Mission Step 5: Perform 720-degree yaw rotation"""
        self.current_state = MissionState.ROTATING
        self.logger.info(f"Performing {self.yaw_rotation}° yaw rotation")
        
        sensor_data = self.sensor_manager.get_sensor_data()
        initial_yaw = sensor_data.yaw
        target_yaw = initial_yaw + self.yaw_rotation
        
        # Normalize target yaw to [-180, 180] range
        while target_yaw > 180:
            target_yaw -= 360
        while target_yaw < -180:
            target_yaw += 360
        
        rotation_timeout = 30  # 30 seconds for rotation
        start_time = time.time()
        
        while (time.time() - start_time) < rotation_timeout:
            sensor_data = self.sensor_manager.get_sensor_data()
            current_yaw = sensor_data.yaw
            
            # Calculate yaw difference
            yaw_diff = target_yaw - current_yaw
            while yaw_diff > 180:
                yaw_diff -= 360
            while yaw_diff < -180:
                yaw_diff += 360
            
            # Check if rotation complete (within 5 degrees)
            if abs(yaw_diff) <= 5:
                self.logger.info("Yaw rotation complete!")
                break
            
            # Calculate yaw control output
            yaw_output = self.pid_controllers.update_yaw_control(current_yaw, target_yaw)
            
            # Maintain depth
            depth_output = self.pid_controllers.update_depth_control(
                sensor_data.depth, self.target_depth
            )
            
            # Apply yaw rotation (differential thrust on horizontal thrusters)
            horizontal_thrust = [
                yaw_output,   # Front-left
                -yaw_output,  # Front-right
                yaw_output,   # Back-left
                -yaw_output   # Back-right
            ]
            
            vertical_thrust = [depth_output] * 4
            
            await self.thruster_controller.set_horizontal_thrust(horizontal_thrust)
            await self.thruster_controller.set_vertical_thrust(vertical_thrust)
            
            self.logger.info(f"Current yaw: {current_yaw:.1f}°, Target: {target_yaw:.1f}°")
            await asyncio.sleep(0.1)
        
        # Stop rotation
        await self.thruster_controller.set_horizontal_thrust([0, 0, 0, 0])
    
    async def _mission_surface(self):
        """Mission Step 6: Surface and stop system"""
        self.current_state = MissionState.SURFACING
        self.logger.info("Surfacing to complete mission")
        
        surface_timeout = 30  # 30 seconds to surface
        start_time = time.time()
        
        while (time.time() - start_time) < surface_timeout:
            sensor_data = self.sensor_manager.get_sensor_data()
            current_depth = sensor_data.depth
            
            # Check if surfaced (depth < 0.2m)
            if current_depth <= 0.2:
                self.logger.info("Surface reached!")
                break
            
            # Apply upward thrust
            upward_thrust = -0.7  # Strong upward thrust
            vertical_thrust = [upward_thrust] * 4
            
            await self.thruster_controller.set_vertical_thrust(vertical_thrust)
            
            self.logger.info(f"Surfacing... Current depth: {current_depth:.2f}m")
            await asyncio.sleep(0.1)
        
        # Stop all thrusters
        await self.thruster_controller.stop_all_thrusters()
        self.logger.info("Mission surfacing complete - All systems stopped")