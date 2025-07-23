"""
Utility functions for aligning AUV with detected objects
"""

import logging
import math

class AlignmentController:
    def __init__(self, frame_width=640, frame_height=480):
        self.logger = logging.getLogger("AlignmentController")
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.center_x = frame_width // 2
        self.center_y = frame_height // 2
        
        # Alignment tolerance in pixels
        self.x_tolerance = 30
        self.y_tolerance = 30
        
    def calculate_alignment_error(self, detection):
        """Calculate alignment error from frame center"""
        if not detection:
            return None
        
        error_x = detection.center[0] - self.center_x
        error_y = detection.center[1] - self.center_y
        
        return error_x, error_y
    
    def is_aligned(self, detection):
        """Check if detection is aligned with frame center"""
        if not detection:
            return False
        
        error_x, error_y = self.calculate_alignment_error(detection)
        
        return (abs(error_x) <= self.x_tolerance and 
                abs(error_y) <= self.y_tolerance)
    
    def get_alignment_commands(self, detection):
        """Get movement commands to align with detection"""
        if not detection:
            return {"horizontal": 0.0, "vertical": 0.0, "forward": 0.0}
        
        error_x, error_y = self.calculate_alignment_error(detection)
        
        # Convert pixel errors to normalized thrust commands
        horizontal_cmd = -error_x / (self.frame_width / 2) * 0.3  # Max 30% thrust
        vertical_cmd = error_y / (self.frame_height / 2) * 0.3    # Max 30% thrust
        
        # Calculate distance-based forward command
        bbox_area = detection.bbox[2] * detection.bbox[3]
        frame_area = self.frame_width * self.frame_height
        relative_size = bbox_area / frame_area
        
        # Move forward if object is small (far away)
        forward_cmd = 0.0
        if relative_size < 0.1:  # Object takes less than 10% of frame
            forward_cmd = 0.4
        elif relative_size > 0.3:  # Object takes more than 30% of frame
            forward_cmd = -0.2
        
        return {
            "horizontal": max(-1.0, min(1.0, horizontal_cmd)),
            "vertical": max(-1.0, min(1.0, vertical_cmd)),
            "forward": max(-1.0, min(1.0, forward_cmd))
        }