"""
Main entry point for ROBOSUB25 AUV System
Initializes all subsystems and executes mission sequence
"""

import asyncio
import logging
import time
from auv.communication.network_manager import NetworkManager
from auv.control.thruster_controller import ThrusterController
from auv.control.pid_controller import PIDController
from auv.sensors.sensor_manager import SensorManager
from auv.vision.object_detector import ObjectDetector
from auv.missions.mission_executor import MissionExecutor
from auv.utils.logger import setup_logger
from config.settings import AUV_CONFIG

class AUVSystem:
    def __init__(self):
        self.logger = setup_logger("AUV_MAIN")
        self.network_manager = None
        self.sensor_manager = None
        self.thruster_controller = None
        self.object_detector = None
        self.mission_executor = None
        self.is_running = False
        
    async def initialize(self):
        """Initialize all AUV subsystems"""
        try:
            self.logger.info("Initializing AUV Systems...")
            
            # Initialize network communication
            self.network_manager = NetworkManager()
            await self.network_manager.initialize()
            
            # Initialize sensors
            self.sensor_manager = SensorManager()
            await self.sensor_manager.initialize()
            
            # Initialize thruster control
            self.thruster_controller = ThrusterController()
            await self.thruster_controller.initialize()
            
            # Initialize vision system
            self.object_detector = ObjectDetector()
            await self.object_detector.initialize()
            
            # Initialize mission executor
            self.mission_executor = MissionExecutor(
                self.sensor_manager,
                self.thruster_controller,
                self.object_detector
            )
            
            self.is_running = True
            self.logger.info("All systems initialized successfully!")
            return True
            
        except Exception as e:
            self.logger.error(f"Initialization failed: {e}")
            return False
    
    async def run_mission(self):
        """Execute the complete mission sequence"""
        if not self.is_running:
            self.logger.error("System not initialized!")
            return
            
        try:
            await self.mission_executor.execute_complete_mission()
        except Exception as e:
            self.logger.error(f"Mission execution failed: {e}")
        finally:
            await self.shutdown()
    
    async def shutdown(self):
        """Gracefully shutdown all systems"""
        self.logger.info("Shutting down AUV systems...")
        
        if self.thruster_controller:
            await self.thruster_controller.stop_all_thrusters()
        
        if self.sensor_manager:
            await self.sensor_manager.shutdown()
            
        if self.network_manager:
            await self.network_manager.shutdown()
            
        self.is_running = False
        self.logger.info("System shutdown complete")

async def main():
    auv = AUVSystem()
    
    if await auv.initialize():
        print("AUV System Ready! Starting Mission...")
        await auv.run_mission()
    else:
        print("Failed to initialize AUV system")

if __name__ == "__main__":
    asyncio.run(main())