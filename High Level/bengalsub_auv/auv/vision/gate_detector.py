"""
Object detection module using YOLOv8 for fish detection
Detects reef_shark and saw_fish classes
"""

import cv2
import numpy as np
import logging
import asyncio
from ultralytics import YOLO
from config.settings import AUV_CONFIG

class DetectedObject:
    def __init__(self, class_name, confidence, bbox, center):
        self.class_name = class_name
        self.confidence = confidence
        self.bbox = bbox  # [x, y, width, height]
        self.center = center  # [x, y]

class ObjectDetector:
    def __init__(self):
        self.logger = logging.getLogger("ObjectDetector")
        self.model = None
        self.class_names = ["reef_shark", "saw_fish"]
        self.is_initialized = False
        
    async def initialize(self):
        """Initialize YOLOv8 model"""
        try:
            # Load YOLOv8 model (assuming model is in auv/vision/models/)
            model_path = "auv/vision/models/fish_detection.pt"
            self.model = YOLO(model_path)
            
            self.is_initialized = True
            self.logger.info("YOLOv8 object detector initialized successfully")
            
        except Exception as e:
            self.logger.error(f"Object detector initialization failed: {e}")
            raise
    
    async def detect_objects(self, frame):
        """
        Detect objects in the given frame
        Returns list of DetectedObject instances
        """
        if not self.is_initialized or frame is None:
            return []
        
        try:
            # Run inference
            results = self.model(frame, verbose=False)
            
            detected_objects = []
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # Extract detection data
                        confidence = float(box.conf[0])
                        class_id = int(box.cls[0])
                        
                        # Filter by confidence threshold
                        if confidence >= AUV_CONFIG["MISSION"]["DETECTION_CONFIDENCE_THRESHOLD"]:
                            # Get bounding box coordinates
                            x1, y1, x2, y2 = box.xyxy[0].tolist()
                            width = x2 - x1
                            height = y2 - y1
                            center_x = x1 + width / 2
                            center_y = y1 + height / 2
                            
                            # Create detected object
                            detected_obj = DetectedObject(
                                class_name=self.class_names[class_id],
                                confidence=confidence,
                                bbox=[x1, y1, width, height],
                                center=[center_x, center_y]
                            )
                            
                            detected_objects.append(detected_obj)
            
            return detected_objects
            
        except Exception as e:
            self.logger.error(f"Object detection error: {e}")
            return []
    
    def get_best_detection(self, detections):
        """Get the detection with highest confidence"""
        if not detections:
            return None
        
        return max(detections, key=lambda d: d.confidence)
    
    def draw_detections(self, frame, detections):
        """Draw detection boxes and labels on frame"""
        annotated_frame = frame.copy()
        
        for detection in detections:
            x, y, w, h = detection.bbox
            
            # Draw bounding box
            cv2.rectangle(annotated_frame, (int(x), int(y)), 
                         (int(x + w), int(y + h)), (0, 255, 0), 2)
            
            # Draw label
            label = f"{detection.class_name}: {detection.confidence:.2f}"
            cv2.putText(annotated_frame, label, (int(x), int(y - 10)),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Draw center point
            cv2.circle(annotated_frame, (int(detection.center[0]), 
                      int(detection.center[1])), 5, (255, 0, 0), -1)
        
        return annotated_frame