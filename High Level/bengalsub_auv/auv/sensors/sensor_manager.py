"""
Sensor manager for AUV - handles all sensor data acquisition
Manages Bar30 depth sensor, VectorNav IMU, and OAK-D camera
"""

import asyncio
import logging
import time
import depthai as dai
import serial
import struct
from config.settings import AUV_CONFIG

class SensorData:
    def __init__(self):
        self.depth = 0.0
        self.pressure = 0.0
        self.temperature = 0.0
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.acceleration = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.timestamp = time.time()

class SensorManager:
    def __init__(self):
        self.logger = logging.getLogger("SensorManager")
        self.sensor_data = SensorData()
        self.is_running = False
        
        # Sensor interfaces
        self.vectornav_serial = None
        self.oak_d_device = None
        self.oak_d_queue = None
        
        # Calibration values for Bar30
        self.surface_pressure = 1013.25  # mbar at sea level
        
    async def initialize(self):
        """Initialize all sensors"""
        try:
            await self._initialize_vectornav()
            await self._initialize_oak_d()
            
            self.is_running = True
            
            # Start sensor reading tasks
            asyncio.create_task(self._read_sensors_loop())
            
            self.logger.info("All sensors initialized successfully")
            
        except Exception as e:
            self.logger.error(f"Sensor initialization failed: {e}")
            raise
    
    async def _initialize_vectornav(self):
        """Initialize VectorNav IMU"""
        try:
            self.vectornav_serial = serial.Serial(
                AUV_CONFIG["SENSORS"]["VECTORNAV_PORT"],
                AUV_CONFIG["SENSORS"]["VECTORNAV_BAUDRATE"],
                timeout=1
            )
            
            # Configure VectorNav for desired output
            self.vectornav_serial.write(b"$VNWRG,07,40*XX\r\n")  # Set output rate to 40Hz
            
            self.logger.info("VectorNav IMU initialized")
            
        except Exception as e:
            self.logger.error(f"VectorNav initialization failed: {e}")
            raise
    
    async def _initialize_oak_d(self):
        """Initialize OAK-D camera"""
        try:
            # Create pipeline
            pipeline = dai.Pipeline()
            
            # Create color camera node
            cam_rgb = pipeline.create(dai.node.ColorCamera)
            cam_rgb.setPreviewSize(*AUV_CONFIG["SENSORS"]["OAK_D_RESOLUTION"])
            cam_rgb.setInterleaved(False)
            cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
            
            # Create output queue
            rgb_out = pipeline.create(dai.node.XLinkOut)
            rgb_out.setStreamName("rgb")
            cam_rgb.preview.link(rgb_out.input)
            
            # Connect to device and start pipeline
            self.oak_d_device = dai.Device(pipeline)
            self.oak_d_queue = self.oak_d_device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            
            self.logger.info("OAK-D camera initialized")
            
        except Exception as e:
            self.logger.error(f"OAK-D initialization failed: {e}")
            raise
    
    async def _read_sensors_loop(self):
        """Main sensor reading loop"""
        while self.is_running:
            try:
                # Read from all sensors
                await self._read_vectornav()
                await self._read_bar30()
                
                # Update timestamp
                self.sensor_data.timestamp = time.time()
                
                # Sleep to maintain update rate
                await asyncio.sleep(1.0 / AUV_CONFIG["SENSORS"]["SENSOR_UPDATE_RATE"])
                
            except Exception as e:
                self.logger.error(f"Sensor reading error: {e}")
                await asyncio.sleep(0.1)
    
    async def _read_vectornav(self):
        """Read data from VectorNav IMU"""
        try:
            if self.vectornav_serial and self.vectornav_serial.in_waiting:
                line = self.vectornav_serial.readline().decode().strip()
                
                if line.startswith("$VNYMR"):
                    # Parse YMR (Yaw, Pitch, Roll) data
                    parts = line.split(',')
                    if len(parts) >= 4:
                        self.sensor_data.yaw = float(parts[1])
                        self.sensor_data.pitch = float(parts[2])
                        self.sensor_data.roll = float(parts[3].split('*')[0])
                        
        except Exception as e:
            self.logger.error(f"VectorNav reading error: {e}")
    
    async def _read_bar30(self):
        """Read data from Bar30 pressure sensor (simulated I2C reading)"""
        try:
            # In a real implementation, this would use I2C to read from Bar30
            # For now, we'll simulate the reading
            
            # Simulated pressure reading (this would be actual I2C communication)
            raw_pressure = 1013.25 + (self.sensor_data.depth * 100)  # Approximate pressure
            
            self.sensor_data.pressure = raw_pressure
            
            # Calculate depth from pressure difference
            pressure_diff = raw_pressure - self.surface_pressure
            self.sensor_data.depth = pressure_diff / 100.0  # Convert mbar to meters (approximate)
            
        except Exception as e:
            self.logger.error(f"Bar30 reading error: {e}")
    
    def get_sensor_data(self):
        """Get current sensor data"""
        return self.sensor_data
    
    def get_camera_frame(self):
        """Get latest camera frame from OAK-D"""
        try:
            if self.oak_d_queue:
                in_rgb = self.oak_d_queue.tryGet()
                if in_rgb is not None:
                    return in_rgb.getCvFrame()
        except Exception as e:
            self.logger.error(f"Camera frame error: {e}")
        return None
    
    def calibrate_depth_sensor(self):
        """Calibrate depth sensor at surface"""
        self.surface_pressure = self.sensor_data.pressure
        self.logger.info(f"Depth sensor calibrated at {self.surface_pressure} mbar")
    
    async def shutdown(self):
        """Shutdown sensor manager"""
        self.is_running = False
        
        if self.vectornav_serial:
            self.vectornav_serial.close()
            
        if self.oak_d_device:
            self.oak_d_device.close()
            
        self.logger.info("Sensor manager shutdown complete")