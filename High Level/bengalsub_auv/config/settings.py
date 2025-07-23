"""
Configuration settings for ROBOSUB25 AUV
"""

AUV_CONFIG = {
    # Network Configuration
    "NETWORK": {
        "JETSON_IP": "192.168.1.100",
        "RASPBERRY_PI_IP": "192.168.1.101", 
        "GROUND_STATION_IP": "192.168.1.10",
        "COMMUNICATION_PORT": 8080,
        "TELEMETRY_PORT": 8081
    },
    
    # Sensor Configuration
    "SENSORS": {
        "BAR30_I2C_ADDRESS": 0x76,
        "VECTORNAV_PORT": "/dev/ttyUSB0",
        "VECTORNAV_BAUDRATE": 115200,
        "OAK_D_RESOLUTION": (640, 480),
        "SENSOR_UPDATE_RATE": 50  # Hz
    },
    
    # Thruster Configuration
    "THRUSTERS": {
        "PWM_MIN": 1100,
        "PWM_MAX": 1900,
        "PWM_NEUTRAL": 1500,
        "VERTICAL_PINS": [18, 19, 20, 21],  # GPIO pins for vertical thrusters
        "HORIZONTAL_PINS": [22, 23, 24, 25],  # GPIO pins for horizontal thrusters
        "PWM_FREQUENCY": 50  # Hz
    },
    
    # Mission Parameters
    "MISSION": {
        "TARGET_DEPTH": 1.5,  # meters
        "STABILIZE_TIME": 5.0,  # seconds
        "YAW_ROTATION": 720.0,  # degrees
        "DETECTION_CONFIDENCE_THRESHOLD": 0.6,
        "MAX_MISSION_TIME": 600  # seconds (10 minutes)
    },
    
    # PID Controller Parameters
    "PID": {
        "DEPTH": {"P": 2.0, "I": 0.1, "D": 0.5},
        "YAW": {"P": 1.5, "I": 0.05, "D": 0.3},
        "PITCH": {"P": 1.0, "I": 0.02, "D": 0.2},
        "ROLL": {"P": 1.0, "I": 0.02, "D": 0.2}
    }
}