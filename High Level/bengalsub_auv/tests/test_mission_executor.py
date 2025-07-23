"""
Test mission execution logic
"""

import unittest
import asyncio
from unittest.mock import Mock, AsyncMock, patch
from auv.missions.mission_executor import MissionExecutor, MissionState
from auv.sensors.sensor_manager import SensorData

class TestMissionExecutor(unittest.TestCase):
    def setUp(self):
        # Create mock components
        self.mock_sensor_manager = Mock()
        self.mock_thruster_controller = Mock()
        self.mock_object_detector = Mock()
        
        # Setup async methods
        self.mock_thruster_controller.set_vertical_thrust = AsyncMock()
        self.mock_thruster_controller.set_horizontal_thrust = AsyncMock()
        self.mock_thruster_controller.stop_all_thrusters = AsyncMock()
        self.mock_object_detector.detect_objects = AsyncMock()
        
        # Create mission executor
        self.mission_executor = MissionExecutor(
            self.mock_sensor_manager,
            self.mock_thruster_controller,
            self.mock_object_detector
        )
    
    def test_mission_state_initialization(self):
        """Test mission executor initialization"""
        self.assertEqual(self.mission_executor.current_state, MissionState.INITIALIZING)
        self.assertFalse(self.mission_executor.is_running)
        self.assertEqual(self.mission_executor.target_depth, 1.5)
    
    async def test_dive_to_depth_success(self):
        """Test successful dive to target depth"""
        # Mock sensor data showing successful depth achievement
        sensor_data = SensorData()
        sensor_data.depth = 1.5
        sensor_data.pitch = 0.0
        sensor_data.roll = 0.0
        
        self.mock_sensor_manager.get_sensor_data.return_value = sensor_data
        
        # Execute dive mission
        await self.mission_executor._mission_dive_to_depth()
        
        # Verify state and thruster calls
        self.assertTrue(self.mock_thruster_controller.set_vertical_thrust.called)
    
    async def test_stabilization_duration(self):
        """Test stabilization maintains for correct duration"""
        sensor_data = SensorData()
        sensor_data.depth = 1.5
        sensor_data.pitch = 0.0
        sensor_data.roll = 0.0
        
        self.mock_sensor_manager.get_sensor_data.return_value = sensor_data
        
        start_time = asyncio.get_event_loop().time()
        await self.mission_executor._mission_stabilize()
        end_time = asyncio.get_event_loop().time()
        
        # Should take approximately the stabilize time (with some tolerance)
        duration = end_time - start_time
        self.assertGreaterEqual(duration, self.mission_executor.stabilize_time - 0.5)