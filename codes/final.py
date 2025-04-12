import cv2
import numpy as np
import time
from ultralytics import YOLO
from Arm_Lib import Arm_Device

time_move = 3000  # Time to move to a position
time_sleep = 1.0
class DofbotTomatoHarvester:
    def __init__(self):
        # Initialize Dofbot arm
        self.arm = Arm_Device()
        
        # Initialize camera (built-in Dofbot camera)
        self.cap = cv2.VideoCapture(0)  # Use appropriate camera index
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Initialize YOLO model for tomato detection with ripeness classes
        self.model = YOLO('best_float32.tflite')  # Replace with path to your trained model
        
        # Define tomato ripeness classes
        self.tomato_classes = {
            0: "ripe",          # Class 0 - Ripe tomato
            1: "unripe",        # Class 1 - Unripe tomato
            2: "unripe",        # Class 2 - Unripe tomato
            3: "ripe",          # Class 3 - Ripe tomato
            4: "unripe",        # Class 4 - Unripe tomato
            5: "unripe"         # Class 5 - Unripe tomato
        }
        
        # Only harvest ripe tomatoes (classes 0 and 3)
        self.ripe_classes = [0, 3]
        
        # Camera intrinsic parameters (calibrate these for your specific camera)
        self.fx = 272  # Focal length in x (pixels)
        self.fy =272  # Focal length in y (pixels)
        self.cx = 320  # Principal point x (typically center of image)
        self.cy = 240  # Principal point y (typically center of image)
        
        # Fixed workspace parameters
        self.fixed_depth = 30.0  # Fixed distance from camera to workspace in cm
        
        # Dofbot arm parameters (based on your DH table)
        self.a1 = 2.5   # Base height (cm)
        self.a2 = 8.1  # Link 2 length (cm)
        self.a3 = 8.3  # Link 3 length (cm)
        self.a4 = 7.4   # Link 4 (rotational joint)
        self.a5 = 10.6   # End-effector length (cm)
        
        # DH parameters (α, a, d) - from your DH table
        self.DH_params = [
            (np.pi/2, 0, self.a1),     # Joint 1
            (0, self.a2, 0),           # Joint 2
            (0, self.a3, 0),           # Joint 3
            (np.pi/2, self.a4, 0),     # Joint 4
            (0, 0, self.a5)            # Joint 5
        ]
        
        # Define home position angles
        ##self.home_position = [90, 90, 90, 90, 90]
        
        # Camera frame transformation
        # This transforms from camera coordinates to robot base coordinates
        # You'll need to calibrate this for your specific setup
        self.camera_to_base = np.array([
            [0, 0, 1, 0],    # Camera X-axis maps to negative robot Y-axis
            [0, 1, 0, 7],     # Camera Y-axis maps to robot X-axis
            [-1, 0, 0, 0],  # Camera Z-axis maps to robot Z-axis, with offset a1
            [0, 0, 0, 1]
        ])
    
    def move_to_home(self):
        """Move the robot arm to home position"""
        #home position
        Arm = Arm_Device()
        Arm.Arm_serial_servo_write(1, 90, time_move)  # Move base
        time.sleep(time_sleep)
        Arm.Arm_serial_servo_write(2, 90, time_move)   # Move shoulder
        time.sleep(time_sleep)
        Arm.Arm_serial_servo_write(3,90, time_move)   # Move elbow
        time.sleep(time_sleep)
        Arm.Arm_serial_servo_write(4, -30, time_move)   # Move wristRobot Base Coordinates
        time.sleep(time_sleep)
        Arm.Arm_serial_servo_write(5, 90, time_move)   # Move wrist
        time.sleep(time_sleep)
        Arm.Arm_serial_servo_write(6, 180, 500)
        print("Robot at home position")
        
    #harvesying position
    def harvest_pos(self):
        Arm = Arm_Device()
        Arm.Arm_serial_servo_write(1, 81, time_move)  # Move base
        time.sleep(time_sleep)
        Arm.Arm_serial_servo_write(6, 50, 500)#gripper
        time.sleep(time_sleep)
        Arm.Arm_serial_servo_write(4, 67, time_move)   # Move wrist
        time.sleep(time_sleep)
        Arm.Arm_serial_servo_write(3, 18, time_move)   # Move elbow
        time.sleep(time_sleep)
        Arm.Arm_serial_servo_write(2, 88, time_move)   # Move shoulder
        time.sleep(time_sleep)
        
    def grab(self):
        Arm = Arm_Device()
        Arm.Arm_serial_servo_write(6, 110, 500)# grab tomato
        time.sleep(time_sleep)
        
    #drop off position
    def drop_off(self):
        Arm = Arm_Device()
        Arm.Arm_serial_servo_write(3, 100, time_move)   # Lift elbow
        time.sleep(time_sleep)
        Arm.Arm_serial_servo_write(1, 180, time_move)   # Move base to drop-off
        time.sleep(time_sleep)
        Arm.Arm_serial_servo_write(2, 70, time_move)   # Move shoulder
        time.sleep(time_sleep)
        Arm.Arm_serial_servo_write(3, 30, time_move)   # Move elbow
        time.sleep(time_sleep)
        Arm.Arm_serial_servo_write(4, 30, time_move)   # Move elbow
        time.sleep(time_sleep)
        
    def release(self):
        Arm = Arm_Device()
        Arm.Arm_serial_servo_write(6, 50, 500)#release obj
        
    def go_back_home(self):
        Arm = Arm_Device()
        Arm.Arm_serial_servo_write(2, 90, time_move)   # Move shoulder
        time.sleep(time_sleep)
        Arm.Arm_serial_servo_write(3,90, time_move)   # Move elbow
        time.sleep(time_sleep)
        Arm.Arm_serial_servo_write(4, -30, time_move)   # Move wristRobot Base Coordinates
        time.sleep(time_sleep)
        Arm.Arm_serial_servo_write(5, 90, time_move)   # Move wrist
        time.sleep(time_sleep)
        Arm.Arm_serial_servo_write(1, 90, time_move)  # Move base
        time.sleep(time_sleep)
        Arm.Arm_serial_servo_write(6, 180, 500)

    
    def detect_tomato(self):
        """Detect tomatoes in camera feed and return position only if ripe tomato found"""
        print("Looking for ripe tomatoes...")
        
        for _ in range(10):  # Try up to 10 frames
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to capture image")
                continue
            
            # Run YOLOv8 inference
            results = self.model.predict(frame, conf=0.25)
            
            # Check if any tomatoes were detected
            if len(results[0].boxes) > 0:
                # Process all detections
                for box in results[0].boxes:
                    class_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    
                    # Get tomato ripeness status
                    ripeness = self.tomato_classes.get(class_id, "unknown")
                    
                    # Get bounding box details
                    x, y, w, h = box.xywh[0]  # Get center x, center y, width, height
                    x, y, w, h = x.item(), y.item(), w.item(), h.item()
                    
                    print(f"Tomato detected at (x, y): ({x:.1f}, {y:.1f}), "
                          f"Width: {w:.1f}, Height: {h:.1f}, "
                          f"Class: {class_id} ({ripeness}), Confidence: {conf:.2f}")
                    
                    # Only process ripe tomatoes (classes 0 and 3)
                    if class_id in self.ripe_classes:
                        print(f"RIPE TOMATO FOUND - Class {class_id} ({ripeness})")
                        return x, y, w, h, class_id
                    else:
                        print(f"Tomato is not ripe yet (Class {class_id} - {ripeness}). Skipping.")
                
                # If we reach here, we found tomatoes but none were ripe
                print("No ripe tomatoes detected. Only unripe tomatoes in frame.")
                return None
        
        print("No tomatoes detected")
        return None
    
    def pixel_to_world_coordinates(self, x_pixel, y_pixel):
        """Convert pixel coordinates to robot base frame coordinates using fixed depth"""
        # Calculate X and Y in camera frame using fixed depth
        X_camera = (x_pixel - self.cx) * self.fixed_depth / self.fx
        Y_camera = (y_pixel - self.cy) * self.fixed_depth / self.fy
        Z_camera = self.fixed_depth
        
        # Create position vector in camera frame
        camera_point = np.array([X_camera, Y_camera, Z_camera, 1.0])
        
        # Transform to robot base frame
        robot_point = self.camera_to_base @ camera_point
        
        X_robot, Y_robot, Z_robot = robot_point[:3]
        
        print(f"Robot Base Coordinates: X: {X_robot:.2f} cm, Y: {Y_robot:.2f} cm, Z: {Z_robot:.2f} cm")
        return X_robot, Y_robot, Z_robot

    def harvest_tomato(self):
        """Complete tomato harvesting sequence with fixed depth approach"""
        try:
            # Step 1: Move to home position
            self.move_to_home()
            
            # Step 2: Detect tomato - only proceed if ripe tomato found
            tomato_data = self.detect_tomato()
            if not tomato_data:
                print("No ripe tomatoes detected. Stopping.")
                return False
                
            x_pixel, y_pixel, width, height, class_id = tomato_data
            
            print(f"Proceeding to harvest RIPE tomato (Class {class_id} - {self.tomato_classes[class_id]})")
            
            # Step 3: Convert to robot base coordinates using fixed depth
            x, y, z = self.pixel_to_world_coordinates(x_pixel, y_pixel)
            
            # Step 4: Calculate inverse kinematics for approach position (slightly above tomato)
            approach_solutions = self.geometric_inverse_kinematics(x, y, z)
            if not approach_solutions:
                print("Cannot reach approach position. Stopping.")
                return False
            
            # Step 5: harvest position
            self.harvest_pos()
            
            # Step 6: grab
            self.grab()
            
            # Step 8: Move to drop off position
            self.drop_off()
            
            # Step 9: release tomato
            self.release()
            
            # Step 10: Move back to approach position
            self.go_back_home()
            print(f"Ripe tomato (Class {class_id}) harvested successfully!")
            return True
            
        except Exception as e:
            print(f"Error during harvesting: {e}")
            import traceback
            traceback.print_exc()
            return False

    def monitor_tomatoes(self):
        """Just monitor tomatoes without harvesting"""
        print("\nMonitoring tomatoes for ripeness...")
        
        try:
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to capture image")
                return
                
            # Run YOLOv8 inference
            results = self.model.predict(frame, conf=0.25)
            
            # Check if any tomatoes were detected
            if len(results[0].boxes) > 0:
                ripe_count = 0
                unripe_count = 0
                
                # Process all detections
                for box in results[0].boxes:
                    class_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    
                    # Get tomato ripeness status
                    ripeness = self.tomato_classes.get(class_id, "unknown")
                    
                    # Get bounding box details
                    x, y, w, h = box.xywh[0]  # Get center x, center y, width, height
                    
                    if class_id in self.ripe_classes:
                        ripe_count += 1
                    else:
                        unripe_count += 1
                        
                    print(f"Tomato detected: Class {class_id} ({ripeness}), Confidence: {conf:.2f}")
                
                print(f"\nRipeness Summary: {ripe_count} ripe, {unripe_count} unripe tomatoes detected")
            else:
                print("No tomatoes detected in frame")
                
        except Exception as e:
            print(f"Error during monitoring: {e}")
    
    def cleanup(self):
        """Release resources"""
        self.cap.release()
        cv2.destroyAllWindows()
        self.move_to_home()
        time.sleep(1)
        print("Cleanup completed")

def main():
    """Main function to run the selective tomato harvesting robot"""
    print("\nYOLO based Tomato Harvester")
    print("This system will only harvest RIPE tomatoes (Classes 0 and 3)")
    
    robot = DofbotTomatoHarvester()
    
    try:
        # Move to home position
        robot.move_to_home()
        
        # Run harvesting loop
        while True:
            print("\nOptions:")
            print("1: Detect and harvest a ripe tomato")
            print("2: Monitor tomatoes (detect without harvesting)")
            print("3: Move to home position")
            print("4: Exit")
            
            choice = input("Enter choice (1-4): ")
            
            if choice == '1':
                print("\nStarting tomato detection and harvesting sequence...")
                success = robot.harvest_tomato()
                if success:
                    print("Harvesting operation completed successfully!")
                else:
                    print("Harvesting operation did not complete - no ripe tomatoes or error occurred.")
            elif choice == '2':
                robot.monitor_tomatoes()
            elif choice == '3':
                robot.move_to_home()
            elif choice == '4':
                break
            else:
                print("Invalid choice. Please try again.")
            
    except KeyboardInterrupt:
        print("\nOperation interrupted by user")
    finally:
        robot.cleanup()
        print("Robot shutdown complete")

if __name__ == "__main__":
    main()