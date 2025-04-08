import numpy as np
import time
from Arm_Lib import Arm_Device

class Kinematic5DOF:
    def __init__(self):
        self.d1 = 3.0    # Base height
        self.a2 = 2.0    # Shoulder to elbow
        self.a3 = 2.0    # Elbow to wrist
        self.d5 = 1.5    # Wrist to end-effector

    def dh_transform(self, theta, alpha, a, d):
        """Returns the homogeneous transformation matrix using DH parameters."""
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, theta):
        """Computes the forward kinematics to get the current end-effector pose."""
        T = np.eye(4)
        DH_params = [
            (-np.pi/2, 0, self.d1),
            (0, self.a2, 0),
            (0, self.a3, 0),
            (np.pi/2, 0, 0),
            (0, 0, self.d5),
        ]
        for i in range(5):
            T = T @ self.dh_transform(theta[i], *DH_params[i])
        position = T[:3, 3]
        orientation = T[:3, :3]
        return position, orientation

    def jacobian(self, theta):
        """Computes the Jacobian matrix for the robotic arm."""
        J = np.zeros((6, 5))
        delta = 1e-6
        pos, _ = self.forward_kinematics(theta)

        for i in range(5):
            theta_temp = np.copy(theta)
            theta_temp[i] += delta
            new_pos, _ = self.forward_kinematics(theta_temp)
            J[:3, i] = (new_pos - pos) / delta
        
        return J

    def inverse_kinematics(self, x, y, z, alpha, beta, gamma, max_iter=100, tolerance=1e-3):
        """Computes inverse kinematics using Jacobian-based Newton-Raphson method."""
        theta = np.radians([0, 45, -45, 0, 0])  # Initial guess
        target_position = np.array([x, y, z])

        for _ in range(max_iter):
            current_position, _ = self.forward_kinematics(theta)
            error = target_position - current_position

            if np.linalg.norm(error) < tolerance:
                return np.degrees(theta)

            J = self.jacobian(theta)

            try:
                J_inv = np.linalg.pinv(J)  # Pseudo-inverse to handle singularities
                delta_theta = J_inv @ error
                theta += delta_theta
            except np.linalg.LinAlgError:
                print("Singularity encountered, stopping computation.")
                return None

        print("IK did not converge.")
        return None

def move_arm(angles):
    """Moves the robotic arm to the given joint angles."""
    Arm = Arm_Device()
    time_move = 1000  # Increased time for smoother motion
    time_sleep = 1.0  # Increased delay for stability

    servo_angles = [
        angles[0],
        90 + angles[1],
        90 + angles[2],
        90 + angles[3],
        angles[4]
    ]

    for i, angle in enumerate(servo_angles):
        Arm.Arm_serial_servo_write(i + 1, angle, time_move)
        time.sleep(time_sleep)
        print(f"Moving Joint {i+1}: {servo_angles[i]}°")

ik_solver = Kinematic5DOF()
x, y, z = 30.0, 10.0, 20.0
alpha, beta, gamma = np.radians(30), np.radians(45), np.radians(60)

solutions = ik_solver.inverse_kinematics(x, y, z, alpha, beta, gamma)
if solutions is not None:
    print("Moving to solution:", solutions)
    move_arm(solutions)
else:
    print("No valid IK solution found.")