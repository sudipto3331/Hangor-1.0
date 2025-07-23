"""
PID Controller for AUV stabilization and movement
Handles depth, yaw, pitch, and roll control
"""

import time
import logging
from config.settings import AUV_CONFIG

class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(-1.0, 1.0)):
        self.logger = logging.getLogger("PIDController")
        
        # PID gains
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # Output limits
        self.output_min, self.output_max = output_limits
        
        # PID state variables
        self.setpoint = 0.0
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
        # Integral windup prevention
        self.integral_max = 1.0
        
    def update(self, current_value, setpoint=None):
        """
        Update PID controller with current measurement
        Returns control output
        """
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0.0:
            return 0.0
        
        # Update setpoint if provided
        if setpoint is not None:
            self.setpoint = setpoint
        
        # Calculate error
        error = self.setpoint - current_value
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term
        self.integral += error * dt
        
        # Prevent integral windup
        if self.integral > self.integral_max:
            self.integral = self.integral_max
        elif self.integral < -self.integral_max:
            self.integral = -self.integral_max
        
        integral_term = self.ki * self.integral
        
        # Derivative term
        derivative = 0.0
        if dt > 0:
            derivative = self.kd * (error - self.last_error) / dt
        
        # Calculate output
        output = proportional + integral_term + derivative
        
        # Apply output limits
        if output > self.output_max:
            output = self.output_max
        elif output < self.output_min:
            output = self.output_min
        
        # Store values for next iteration
        self.last_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        """Reset PID controller state"""
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
    
    def set_gains(self, kp, ki, kd):
        """Update PID gains"""
        self.kp = kp
        self.ki = ki
        self.kd = kd


class AUVPIDControllers:
    def __init__(self):
        self.logger = logging.getLogger("AUVPIDControllers")
        
        # Initialize PID controllers for each axis
        depth_pid = AUV_CONFIG["PID"]["DEPTH"]
        self.depth_controller = PIDController(
            depth_pid["P"], depth_pid["I"], depth_pid["D"]
        )
        
        yaw_pid = AUV_CONFIG["PID"]["YAW"]
        self.yaw_controller = PIDController(
            yaw_pid["P"], yaw_pid["I"], yaw_pid["D"]
        )
        
        pitch_pid = AUV_CONFIG["PID"]["PITCH"]
        self.pitch_controller = PIDController(
            pitch_pid["P"], pitch_pid["I"], pitch_pid["D"]
        )
        
        roll_pid = AUV_CONFIG["PID"]["ROLL"]
        self.roll_controller = PIDController(
            roll_pid["P"], roll_pid["I"], roll_pid["D"]
        )
    
    def update_depth_control(self, current_depth, target_depth):
        """Update depth control and return vertical thrust command"""
        return self.depth_controller.update(current_depth, target_depth)
    
    def update_yaw_control(self, current_yaw, target_yaw):
        """Update yaw control and return yaw thrust command"""
        # Normalize angle difference to [-180, 180]
        angle_diff = target_yaw - current_yaw
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        
        # Use angle difference as error for PID
        return self.yaw_controller.update(0, angle_diff)
    
    def update_pitch_control(self, current_pitch, target_pitch=0.0):
        """Update pitch control and return pitch thrust command"""
        return self.pitch_controller.update(current_pitch, target_pitch)
    
    def update_roll_control(self, current_roll, target_roll=0.0):
        """Update roll control and return roll thrust command"""
        return self.roll_controller.update(current_roll, target_roll)
    
    def reset_all_controllers(self):
        """Reset all PID controllers"""
        self.depth_controller.reset()
        self.yaw_controller.reset()
        self.pitch_controller.reset()
        self.roll_controller.reset()