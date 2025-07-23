import depthai as dai
import cv2
import numpy as np
from ultralytics import YOLO

# Load YOLOv8 model
model = YOLO("D:\\Competition\\RoboSub'25\\RoboSub25\\bengalsub_auv\\auv\\vision\\models\\gate_model_test1.pt") 

# Create DepthAI pipeline
pipeline = dai.Pipeline()

# Color camera
cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setPreviewSize(640, 480)
cam_rgb.setInterleaved(False)
cam_rgb.setFps(30)

# Mono cameras for depth
mono_left = pipeline.create(dai.node.MonoCamera)
mono_right = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)

mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

# Link mono to stereo depth
mono_left.out.link(stereo.left)
mono_right.out.link(stereo.right)

# Depth output
xout_depth = pipeline.create(dai.node.XLinkOut)
xout_depth.setStreamName("depth")
stereo.depth.link(xout_depth.input)

# RGB output
xout_rgb = pipeline.create(dai.node.XLinkOut)
xout_rgb.setStreamName("rgb")
cam_rgb.preview.link(xout_rgb.input)

# Start device
with dai.Device(pipeline) as device:
    q_rgb = device.getOutputQueue("rgb", maxSize=4, blocking=False)
    q_depth = device.getOutputQueue("depth", maxSize=4, blocking=False)

    while True:
        rgb_frame = q_rgb.get().getCvFrame()
        depth_frame = q_depth.get().getFrame()  # 16-bit depth map

        # Normalize depth for visualization
        depth_vis = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX)
        depth_vis = cv2.convertScaleAbs(depth_vis)

        # Run YOLOv8 inference
        results = model(rgb_frame, conf=0.6)[0]

        # Plot detections
        annotated = results.plot()

        # Find the best (highest confidence) prediction
        if len(results.boxes) > 0:
            best_box = results.boxes[0]  # First one is best (sorted by confidence)
            xyxy = best_box.xyxy.cpu().numpy().astype(int)[0]
            x1, y1, x2, y2 = xyxy
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

            # Get depth at the center of the box
            if 0 <= cx < depth_frame.shape[1] and 0 <= cy < depth_frame.shape[0]:
                distance_mm = depth_frame[cy, cx]
                distance_m = distance_mm / 1000.0  # convert to meters

                # Draw distance on frame
                cv2.putText(
                    annotated,
                    f"Distance: {distance_m:.2f} m",
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                )
                print(f"[INFO] Detected '{best_box.cls}' at distance: {distance_m:.2f} meters")

        # Display output
        cv2.imshow("RGB + YOLOv8", annotated)
        cv2.imshow("Depth", depth_vis)

        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()
