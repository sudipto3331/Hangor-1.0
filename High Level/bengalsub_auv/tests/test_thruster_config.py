"""
Test thruster configuration and movements
"""

import unittest
import asyncio
from unittest.mock import Mock, patch
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from auv.control.thruster_controller import ThrusterController

class TestThrusterConfiguration(unittest.TestCase):
    def setUp(self):
        self.controller = ThrusterController()
    
    @patch('RPi.GPIO.PWM')
    @patch('RPi.GPIO.setup')
    @patch('RPi.GPIO.setmode')
    async def test_thruster_initialization(self, mock_setmode, mock_setup, mock_pwm):
        """Test thruster controller initialization"""
        mock_pwm_instance = Mock()
        mock_pwm.return_value = mock_pwm_instance
        
        await self.controller.initialize()
        
        self.assertTrue(self.controller.is_initialized)
        self.assertEqual(len(self.controller.vertical_thrusters), 4)
        self.assertEqual(len(self.controller.horizontal_thrusters), 4)
    
    def test_pwm_conversion(self):
        """Test PWM value to duty cycle conversion"""
        # Test neutral position
        neutral_duty = self.controller._pwm_to_duty_cycle(1500)
        self.assertAlmostEqual(neutral_duty, 7.5, places=1)  # 1.5ms / 20ms * 100
        
        # Test minimum PWM
        min_duty = self.controller._pwm_to_duty_cycle(1100)
        self.assertAlmostEqual(min_duty, 5.5, places=1)
        
        # Test maximum PWM
        max_duty = self.controller._pwm_to_duty_cycle(1900)
        self.assertAlmostEqual(max_duty, 9.5, places=1)
    
    def test_pwm_constraints(self):
        """Test PWM value constraints"""
        # Test value above maximum
        constrained = self.controller._constrain_pwm(2000)
        self.assertEqual(constrained, 1900)
        
        # Test value below minimum
        constrained = self.controller._constrain_pwm(1000)
        self.assertEqual(constrained, 1100)
        
        # Test normal value
        constrained = self.controller._constrain_pwm(1500)
        self.assertEqual(constrained, 1500)