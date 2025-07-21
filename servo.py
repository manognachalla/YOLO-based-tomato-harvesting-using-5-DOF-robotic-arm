#!/usr/bin/env python3
#coding=utf-8
import time
from Arm_Lib import Arm_Device

# Get DOFBOT object
Arm = Arm_Device()
time.sleep(.1)

# --- CONFIGURATION PARAMETERS (YOU MUST ADJUST THESE!) ---
# Define the servo angles for opening and closing the gripper (Servo ID 6)
# Experiment with your DOFBOT to find these exact values.
GRIPPER_OPEN_ANGLE = 20  # Example: 90 degrees might be wide open
GRIPPER_CLOSE_ANGLE = 150 # Example: 40 degrees might be closed enough to grip (adjust carefully!)

# Define the full set of 6 servo angles for different arm positions
# These are crucial and need to be determined by manually moving your arm
# to the desired positions and reading the current servo angles (if Arm_Lib supports it),
# or by careful trial and error.

# Example angles for a 'home' or 'ready' position (all joints centered/neutral)
HOME_POS = [90, 90, 90, 90, 90, GRIPPER_OPEN_ANGLE]

# Example angles for the PICK-UP location (above the object)
# Adjust these based on your physical setup. Servo 6 is always the gripper here.
PICK_LOCATION_ANGLES_HOVER = [90, 60, 120, 0, 90, GRIPPER_OPEN_ANGLE] # Arm position above pick-up
PICK_LOCATION_ANGLES_DOWN = [90, 40, 140, 0, 90, GRIPPER_OPEN_ANGLE] # Arm position at pick-up height

# Example angles for the PLACE-DOWN location (above the drop-off point)
# Adjust these based on your physical setup. Servo 6 is always the gripper here.
PLACE_LOCATION_ANGLES_HOVER = [150, 60, 120, 0, 90, GRIPPER_CLOSE_ANGLE] # Arm position above place-down
PLACE_LOCATION_ANGLES_DOWN = [150, 40, 140, 0, 90, GRIPPER_CLOSE_ANGLE] # Arm position at place-down height


# --- HELPER FUNCTION FOR COORDIANTED MOVEMENT ---
# This function sends angles to all 6 servos and waits for the movement duration.
# Servo IDs are assumed to be 1, 2, 3, 4, 5, 6 in order.
# The `Arm_serial_servo_write6` function usually takes 6 angles, then the time.
# The commented out `ctrl_all_servo` from your original code wasn't general
# enough for arbitrary poses, so we'll use `Arm.Arm_serial_servo_write6` directly.
def move_arm_to_pose(angles, duration_ms):
    """
    Moves the robotic arm to a specified pose (6 joint angles).

    Args:
        angles (list): A list of 6 integer angles [s1_angle, s2_angle, ..., s6_angle].
        duration_ms (int): The time in milliseconds for the movement to complete.
    """
    if len(angles) != 6:
        print("Error: move_arm_to_pose requires exactly 6 angles.")
        return

    # Use the library's function to write to all 6 servos
    Arm.Arm_serial_servo_write6(
        angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], duration_ms
    )
    # Wait for the movement to finish
    time.sleep(duration_ms / 1000.0) # Convert ms to seconds for time.sleep


# --- MAIN PICK AND PLACE LOGIC ---
def pick_and_place_routine():
    print("Starting Pick and Place Routine...")

    # 1. Go to Home/Ready Position (with gripper open)
    print("Moving to Home Position...")
    move_arm_to_pose(HOME_POS, 1000) # 1 second duration
    time.sleep(0.5) # Short pause after reaching home

    while True: # Loop for continuous pick and place
        # --- PICK OPERATION ---
        print("\n--- PICKING ---")

        # 2. Move to hover position above pick-up
        print("Moving to Pick Hover Position...")
        move_arm_to_pose(PICK_LOCATION_ANGLES_HOVER, 1000)
        time.sleep(0.5)

        # 3. Open gripper (ensure it's open before descending)
        print("Ensuring Gripper is Open...")
        # Only change servo 6 (gripper), keep other angles the same as current hover position
        current_angles = list(PICK_LOCATION_ANGLES_HOVER) # Make a copy
        current_angles[5] = GRIPPER_OPEN_ANGLE # Servo 6 is at index 5
        move_arm_to_pose(current_angles, 200) # Quick gripper open
        time.sleep(0.2)

        # 4. Descend to pick-up height
        print("Descending to Pick-up Height...")
        # The last angle is GRIPPER_OPEN_ANGLE
        move_arm_to_pose(PICK_LOCATION_ANGLES_DOWN, 500)
        time.sleep(0.5)

        # 5. Close gripper to grab object
        print("Closing Gripper to Grab...")
        current_angles = list(PICK_LOCATION_ANGLES_DOWN) # Make a copy of the down pose
        current_angles[5] = GRIPPER_CLOSE_ANGLE # Set gripper to closed
        move_arm_to_pose(current_angles, 500) # Close gripper
        time.sleep(1) # Give it time to grip firmly

        # 6. Lift arm to clear object
        print("Lifting Object...")
        # Move back to hover position, keeping gripper closed
        current_angles = list(PICK_LOCATION_ANGLES_HOVER)
        current_angles[5] = GRIPPER_CLOSE_ANGLE # Ensure gripper remains closed
        move_arm_to_pose(current_angles, 700)
        time.sleep(0.5)


        # --- PLACE OPERATION ---
        print("\n--- PLACING ---")

        # 7. Move to hover position above place-down
        print("Moving to Place Hover Position...")
        move_arm_to_pose(PLACE_LOCATION_ANGLES_HOVER, 1500) # Longer move duration
        time.sleep(0.5)

        # 8. Descend to place-down height
        print("Descending to Place-down Height...")
        # The last angle is GRIPPER_CLOSE_ANGLE
        move_arm_to_pose(PLACE_LOCATION_ANGLES_DOWN, 500)
        time.sleep(0.5)

        # 9. Open gripper to release object
        print("Opening Gripper to Release...")
        current_angles = list(PLACE_LOCATION_ANGLES_DOWN)
        current_angles[5] = GRIPPER_OPEN_ANGLE # Set gripper to open
        move_arm_to_pose(current_angles, 500) # Open gripper
        time.sleep(0.5) # Give it time to release

        # 10. Lift arm after releasing
        print("Lifting Arm After Release...")
        current_angles = list(PLACE_LOCATION_ANGLES_HOVER)
        current_angles[5] = GRIPPER_OPEN_ANGLE # Ensure gripper remains open
        move_arm_to_pose(current_angles, 700)
        time.sleep(0.5)

        # 11. Return to Home/Ready Position
        print("Returning to Home Position...")
        move_arm_to_pose(HOME_POS, 1000)
        time.sleep(1)

        print("\n--- Pick and Place Cycle Complete ---")
        time.sleep(2) # Pause before starting next cycle (optional)


# --- MAIN EXECUTION BLOCK ---
try:
    pick_and_place_routine()
except KeyboardInterrupt:
    print("\nProgram closed by user (KeyboardInterrupt)!")
    pass
finally:
    # Optional: Move arm to a safe, neutral position on exit
    print("Moving arm to final safe position...")
    Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, GRIPPER_OPEN_ANGLE, 1000)
    time.sleep(1)
    del Arm  # Release DOFBOT object
    print("Resources released.")
