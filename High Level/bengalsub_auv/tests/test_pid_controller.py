"""
Test PID controller functionality
"""

import unittest
import time
from auv.control.pid_controller import PIDController

class TestPIDController(unittest.TestCase):
    def setUp(self):
        self.pid = PIDController(kp=1.0, ki=0.1, kd=0.01)
    
    def test_proportional_control(self):
        """Test proportional term calculation"""
        # Set a target and test response
        self.pid.setpoint = 10.0
        output = self.pid.update(8.0)  # Error = 2.0
        
        # Should have positive output for positive error
        self.assertGreater(output, 0)
    
    def test_integral_windup_prevention(self):
        """Test integral windup prevention"""
        self.pid.setpoint = 10.0
        
        # Accumulate large integral error
        for _ in range(100):
            self.pid.update(0.0)  # Large persistent error
            time.sleep(0.01)
        
        # Integral should be limited
        self.assertLessEqual(abs(self.pid.integral), self.pid.integral_max)
    
    def test_output_limits(self):
        """Test output limiting"""
        self.pid.kp = 10.0  # High gain to force saturation
        self.pid.setpoint = 100.0
        
        output = self.pid.update(0.0)  # Large error
        
        # Output should be limited to range
        self.assertGreaterEqual(output, self.pid.output_min)
        self.assertLessEqual(output, self.pid.output_max)
    
    def test_reset_functionality(self):
        """Test PID reset"""
        # Generate some state
        self.pid.setpoint = 10.0
        self.pid.update(5.0)
        
        # Reset and verify state is cleared
        self.pid.reset()
        
        self.assertEqual(self.pid.last_error, 0.0)
        self.assertEqual(self.pid.integral, 0.0)
