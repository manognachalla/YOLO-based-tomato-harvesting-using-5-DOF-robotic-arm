import numpy as np

# Camera intrinsic parameters
fx=fy=600  # Focal lengths in pixels
cx, cy = 320, 240        # Principal point
Z = 300                  # Fixed depth in mm

# Example YOLO output: [(x, y, width, height)]
yolo_detections = [
    (334,335,242,228),  # Example detection 1
]

# Convert detections
for (x, y, w, h) in yolo_detections:
    # Convert pixel coordinates to real-world coordinates
    X = (x - cx) * Z / fx
    Y = (y - cy) * Z / fy

    # Estimate real-world width & height
    real_width = (w * Z) / fx
    real_height = (h * Z) / fy

    print(f"Detected Tomato at (pixel): ({x}, {y}), bbox: ({w}, {h})")
    print(f"Real-World X: {X:.2f} mm")
    print(f"Real-World Y: {Y:.2f} mm")
    print(f"Real-World Width: {real_width:.2f} mm")
    print(f"Real-World Height: {real_height:.2f} mm")
    print(f"Fixed Depth (Z): {Z:.2f} mm\n")