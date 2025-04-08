import cv2
import time
from ultralytics import YOLO
import numpy as np

# Load YOLO model
try:
    model = YOLO('best_float32.tflite')
    print("Model loaded successfully")
except Exception as e:
    print(f"Error loading model: {e}")
    exit()

# Try different camera indices if needed
camera_index = 4
max_attempts = 5
attempt = 0

while attempt < max_attempts:
    try:
        cap = cv2.VideoCapture(camera_index)
        if cap.isOpened():
            print(f"Camera {camera_index} opened successfully")
            break
        else:
            print(f"Failed to open camera {camera_index}")
            camera_index = (camera_index + 1) % 5  # Try next index (0-4)
            attempt += 1
    except Exception as e:
        print(f"Error opening camera {camera_index}: {e}")
        camera_index = (camera_index + 1) % 5
        attempt += 1

if not cap.isOpened():
    print("Could not open any camera")
    exit()

# Reduce resolution for better performance
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 10)

# FPS calculation variables
frame_count = 0
start_time = time.time()

try:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        # Calculate FPS
        frame_count += 1
        elapsed_time = time.time() - start_time
        if elapsed_time > 1:
            fps = frame_count / elapsed_time
            print(f"FPS: {fps:.2f}")
            frame_count = 0
            start_time = time.time()
        
        try:
            # Run YOLO inference
            results = model(frame)
            
            # Process results and overlay on frame
            for r in results:
                frame = r.plot()  # Draw detections
                
                # Extract bounding boxes
                if hasattr(r.boxes, 'xyxy'):
                    for box in r.boxes.xyxy:
                        x1, y1, x2, y2 = box.cpu().numpy() if hasattr(box, 'cpu') else box
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)
                        print(f"Tomato detected at: ({center_x}, {center_y})")
                        # Draw center point
                        cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
                else:
                    print("No 'xyxy' attribute in boxes")
        except Exception as e:
            print(f"Error during detection: {e}")
            
        # Show output
        cv2.imshow("YOLO Camera Detection", frame)
        
        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    print("Interrupted by user")
except Exception as e:
    print(f"Unexpected error: {e}")
finally:
    # Clean up
    cap.release()
    cv2.destroyAllWindows()
    print("Resources released")