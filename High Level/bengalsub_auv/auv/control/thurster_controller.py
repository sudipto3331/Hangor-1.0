"""
Thruster controller for AUV - controls 8 thrusters (4 vertical, 4 horizontal)
Uses PWM signals to control ESCs
"""

import asyncio
import logging
import RPi.GPIO as GPIO
from config.settings import AUV_CONFIG

class ThrusterController:
    def __init__(self):
        self.logger = logging.getLogger("ThrusterController")
        self.vertical_thrusters = []  # [front-left, front-right, back-left, back-right]
        self.horizontal_thrusters = []  # [front-left, front-right, back-left, back-right]
        self.is_initialized = False
        
    async def initialize(self):
        """Initialize thruster controller and PWM signals"""
        try:
            # Setup GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Initialize vertical thrusters (pointing downward)
            for pin in AUV_CONFIG["THRUSTERS"]["VERTICAL_PINS"]:
                GPIO.setup(pin, GPIO.OUT)
                pwm = GPIO.PWM(pin, AUV_CONFIG["THRUSTERS"]["PWM_FREQUENCY"])
                pwm.start(self._pwm_to_duty_cycle(AUV_CONFIG["THRUSTERS"]["PWM_NEUTRAL"]))
                self.vertical_thrusters.append(pwm)
            
            # Initialize horizontal thrusters (45-degree angle)
            for pin in AUV_CONFIG["THRUSTERS"]["HORIZONTAL_PINS"]:
                GPIO.setup(pin, GPIO.OUT)
                pwm = GPIO.PWM(pin, AUV_CONFIG["THRUSTERS"]["PWM_FREQUENCY"])
                pwm.start(self._pwm_to_duty_cycle(AUV_CONFIG["THRUSTERS"]["PWM_NEUTRAL"]))
                self.horizontal_thrusters.append(pwm)
            
            # Wait for ESCs to initialize
            await asyncio.sleep(2)
            
            self.is_initialized = True
            self.logger.info("Thruster controller initialized successfully")
            
        except Exception as e:
            self.logger.error(f"Thruster initialization failed: {e}")
            raise
    
    def _pwm_to_duty_cycle(self, pwm_value):
        """Convert PWM value (1100-1900) to duty cycle percentage"""
        period_ms = 1000 / AUV_CONFIG["THRUSTERS"]["PWM_FREQUENCY"]
        pulse_width_ms = pwm_value / 1000.0
        return (pulse_width_ms / period_ms) * 100
    
    def _constrain_pwm(self, pwm_value):
        """Constrain PWM value to safe limits"""
        return max(
            AUV_CONFIG["THRUSTERS"]["PWM_MIN"],
            min(AUV_CONFIG["THRUSTERS"]["PWM_MAX"], pwm_value)
        )
    
    async def set_vertical_thrust(self, thrust_values):
        """
        Set vertical thruster values
        thrust_values: [front-left, front-right, back-left, back-right] (-1.0 to 1.0)
        Positive values = downward thrust, Negative values = upward thrust
        """
        if not self.is_initialized:
            self.logger.error("Thruster controller not initialized")
            return
        
        try:
            for i, thrust in enumerate(thrust_values):
                # Convert thrust (-1.0 to 1.0) to PWM value
                thrust = max(-1.0, min(1.0, thrust))
                
                # Apply rotation direction based on thruster configuration
                if i in [0, 3]:  # Front-left and back-right (clockwise)
                    pwm_value = AUV_CONFIG["THRUSTERS"]["PWM_NEUTRAL"] + (thrust * 400)
                else:  # Front-right and back-left (counter-clockwise)
                    pwm_value = AUV_CONFIG["THRUSTERS"]["PWM_NEUTRAL"] - (thrust * 400)
                
                pwm_value = self._constrain_pwm(pwm_value)
                duty_cycle = self._pwm_to_duty_cycle(pwm_value)
                
                self.vertical_thrusters[i].ChangeDutyCycle(duty_cycle)
                
        except Exception as e:
            self.logger.error(f"Vertical thrust control error: {e}")
    
    async def set_horizontal_thrust(self, thrust_values):
        """
        Set horizontal thruster values
        thrust_values: [front-left, front-right, back-left, back-right] (-1.0 to 1.0)
        """
        if not self.is_initialized:
            self.logger.error("Thruster controller not initialized")
            return
        
        try:
            for i, thrust in enumerate(thrust_values):
                # Convert thrust (-1.0 to 1.0) to PWM value
                thrust = max(-1.0, min(1.0, thrust))
                
                # Apply rotation direction based on thruster configuration
                if i in [0, 1]:  # Front thrusters (clockwise)
                    pwm_value = AUV_CONFIG["THRUSTERS"]["PWM_NEUTRAL"] + (thrust * 400)
                else:  # Back thrusters (counter-clockwise)
                    pwm_value = AUV_CONFIG["THRUSTERS"]["PWM_NEUTRAL"] - (thrust * 400)
                
                pwm_value = self._constrain_pwm(pwm_value)
                duty_cycle = self._pwm_to_duty_cycle(pwm_value)
                
                self.horizontal_thrusters[i].ChangeDutyCycle(duty_cycle)
                
        except Exception as e:
            self.logger.error(f"Horizontal thrust control error: {e}")
    
    async def move_forward(self, thrust_level=0.5):
        """Move AUV forward"""
        thrust_values = [thrust_level, thrust_level, thrust_level, thrust_level]
        await self.set_horizontal_thrust(thrust_values)
    
    async def move_backward(self, thrust_level=0.5):
        """Move AUV backward"""
        thrust_values = [-thrust_level, -thrust_level, -thrust_level, -thrust_level]
        await self.set_horizontal_thrust(thrust_values)
    
    async def move_left(self, thrust_level=0.5):
        """Move AUV left (strafe)"""
        thrust_values = [-thrust_level, thrust_level, thrust_level, -thrust_level]
        await self.set_horizontal_thrust(thrust_values)
    
    async def move_right(self, thrust_level=0.5):
        """Move AUV right (strafe)"""
        thrust_values = [thrust_level, -thrust_level, -thrust_level, thrust_level]
        await self.set_horizontal_thrust(thrust_values)
    
    async def rotate_yaw(self, thrust_level=0.5):
        """Rotate AUV in yaw (around vertical axis)"""
        thrust_values = [thrust_level, -thrust_level, thrust_level, -thrust_level]
        await self.set_horizontal_thrust(thrust_values)
    
    async def move_up(self, thrust_level=0.5):
        """Move AUV upward"""
        thrust_values = [-thrust_level, -thrust_level, -thrust_level, -thrust_level]
        await self.set_vertical_thrust(thrust_values)
    
    async def move_down(self, thrust_level=0.5):
        """Move AUV downward"""
        thrust_values = [thrust_level, thrust_level, thrust_level, thrust_level]
        await self.set_vertical_thrust(thrust_values)
    
    async def stabilize_depth(self, thrust_level=0.3):
        """Apply minimal thrust to maintain depth"""
        thrust_values = [thrust_level, thrust_level, thrust_level, thrust_level]
        await self.set_vertical_thrust(thrust_values)
    
    async def stop_all_thrusters(self):
        """Stop all thrusters - set to neutral position"""
        neutral_vertical = [0.0, 0.0, 0.0, 0.0]
        neutral_horizontal = [0.0, 0.0, 0.0, 0.0]
        
        await self.set_vertical_thrust(neutral_vertical)
        await self.set_horizontal_thrust(neutral_horizontal)
        
        self.logger.info("All thrusters stopped")
    
    async def shutdown(self):
        """Shutdown thruster controller"""
        await self.stop_all_thrusters()
        
        # Stop all PWM signals
        for pwm in self.vertical_thrusters:
            pwm.stop()
        for pwm in self.horizontal_thrusters:
            pwm.stop()
        
        GPIO.cleanup()
        self.logger.info("Thruster controller shutdown complete")