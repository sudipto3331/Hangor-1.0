"""
Test vision detection and alignment
"""

import unittest
import numpy as np
from unittest.mock import Mock, patch
from auv.vision.object_detector import DetectedObject
from auv.vision.utils.alignment import AlignmentController

class TestVisionSystem(unittest.TestCase):
    def setUp(self):
        self.alignment_controller = AlignmentController(640, 480)
    
    def test_detection_object_creation(self):
        """Test DetectedObject creation"""
        detection = DetectedObject(
            class_name="reef_shark",
            confidence=0.85,
            bbox=[100, 100, 200, 150],
            center=[200, 175]
        )
        
        self.assertEqual(detection.class_name, "reef_shark")
        self.assertEqual(detection.confidence, 0.85)
        self.assertEqual(detection.center, [200, 175])
    
    def test_alignment_error_calculation(self):
        """Test alignment error calculation"""
        # Create detection at frame center
        center_detection = DetectedObject(
            class_name="test",
            confidence=0.8,
            bbox=[0, 0, 100, 100],
            center=[320, 240]  # Frame center
        )
        
        error_x, error_y = self.alignment_controller.calculate_alignment_error(center_detection)
        self.assertEqual(error_x, 0)
        self.assertEqual(error_y, 0)
        
        # Create detection off-center
        off_center_detection = DetectedObject(
            class_name="test",
            confidence=0.8,
            bbox=[0, 0, 100, 100],
            center=[420, 340]  # Off-center
        )
        
        error_x, error_y = self.alignment_controller.calculate_alignment_error(off_center_detection)
        self.assertEqual(error_x, 100)  # 420 - 320
        self.assertEqual(error_y, 100)  # 340 - 240
    
    def test_alignment_commands(self):
        """Test alignment command generation"""
        # Detection to the right and below center
        detection = DetectedObject(
            class_name="test",
            confidence=0.8,
            bbox=[0, 0, 50, 50],  # Small object (far away)
            center=[420, 340]
        )
        
        commands = self.alignment_controller.get_alignment_commands(detection)
        
        # Should command leftward movement (negative horizontal)
        self.assertLess(commands["horizontal"], 0)
        
        # Should command upward movement (positive vertical)
        self.assertGreater(commands["vertical"], 0)
        
        # Should command forward movement (small object)
        self.assertGreater(commands["forward"], 0)
    
    def test_is_aligned(self):
        """Test alignment detection"""
        # Perfectly aligned detection
        aligned_detection = DetectedObject(
            class_name="test",
            confidence=0.8,
            bbox=[0, 0, 100, 100],
            center=[320, 240]
        )
        
        self.assertTrue(self.alignment_controller.is_aligned(aligned_detection))
        
        # Misaligned detection
        misaligned_detection = DetectedObject(
            class_name="test",
            confidence=0.8,
            bbox=[0, 0, 100, 100],
            center=[400, 300]
        )
        
        self.assertFalse(self.alignment_controller.is_aligned(misaligned_detection))