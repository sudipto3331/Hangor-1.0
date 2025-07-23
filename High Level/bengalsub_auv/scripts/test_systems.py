"""
System test script for AUV components
Tests individual subsystems before mission
"""

import sys
import os
import asyncio

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from auv.sensors.sensor_manager import SensorManager
from auv.control.thruster_controller import ThrusterController
from auv.vision.gate_detector_detector import ObjectDetector
from auv.utils.logger import setup_logger

async def test_sensors():
    """Test sensor functionality"""
    logger = setup_logger("SensorTest")
    logger.info("Testing sensors...")
    
    try:
        sensor_manager = SensorManager()
        await sensor_manager.initialize()
        
        # Test sensor readings
        for i in range(10):
            sensor_data = sensor_manager.get_sensor_data()
            logger.info(f"Depth: {sensor_data.depth:.2f}m, "
                       f"Yaw: {sensor_data.yaw:.1f}°, "
                       f"Pitch: {sensor_data.pitch:.1f}°, "
                       f"Roll: {sensor_data.roll:.1f}°")
            await asyncio.sleep(1)
        
        await sensor_manager.shutdown()
        logger.info("Sensor test completed successfully")
        return True
        
    except Exception as e:
        logger.error(f"Sensor test failed: {e}")
        return False

async def test_thrusters():
    """Test thruster functionality"""
    logger = setup_logger("ThrusterTest")
    logger.info("Testing thrusters...")
    
    try:
        thruster_controller = ThrusterController()
        await thruster_controller.initialize()
        
        # Test thruster movements
        logger.info("Testing vertical thrusters...")
        await thruster_controller.move_down(0.3)
        await asyncio.sleep(2)
        await thruster_controller.move_up(0.3)
        await asyncio.sleep(2)
        
        logger.info("Testing horizontal thrusters...")
        await thruster_controller.move_forward(0.3)
        await asyncio.sleep(2)
        await thruster_controller.move_backward(0.3)
        await asyncio.sleep(2)
        
        await thruster_controller.stop_all_thrusters()
        await thruster_controller.shutdown()
        logger.info("Thruster test completed successfully")
        return True
        
    except Exception as e:
        logger.error(f"Thruster test failed: {e}")
        return False

async def test_vision():
    """Test vision system"""
    logger = setup_logger("VisionTest")
    logger.info("Testing vision system...")
    
    try:
        object_detector = ObjectDetector()
        await object_detector.initialize()
        
        logger.info("Vision system initialized - ready for detection")
        # Note: Actual detection would require camera feed
        
        logger.info("Vision test completed successfully")
        return True
        
    except Exception as e:
        logger.error(f"Vision test failed: {e}")
        return False

async def main():
    """Run all system tests"""
    logger = setup_logger("SystemTest")
    logger.info("Starting AUV system tests...")
    
    results = {
        "sensors": await test_sensors(),
        "thrusters": await test_thrusters(),
        "vision": await test_vision()
    }
    
    logger.info("Test Results:")
    for system, result in results.items():
        status = "PASS" if result else "FAIL"
        logger.info(f"  {system.upper()}: {status}")
    
    all_passed = all(results.values())
    if all_passed:
        logger.info("All systems tests PASSED - AUV ready for mission")
    else:
        logger.error("Some system tests FAILED - Check issues before mission")
    
    return all_passed

if __name__ == "__main__":
    asyncio.run(main())