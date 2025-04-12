# ik_solver.py
#also includes joint angle to servo angle conversion
import numpy as np
import sympy as sp

# Define symbolic variables
theta1, theta2, theta3, theta4, theta5 = sp.symbols('theta1 theta2 theta3 theta4 theta5', real=True)
L1, L2, L3, L4, L5 = sp.symbols('L1 L2 L3 L4 L5', real=True)

# Store in global variables
theta_syms = sp.Matrix([theta1, theta2, theta3, theta4, theta5])
length_syms = sp.Matrix([L1, L2, L3, L4, L5])

def Rx(phi):
    """Rotation matrix around x-axis"""
    return sp.Matrix([
        [1, 0, 0],
        [0, sp.cos(phi), -sp.sin(phi)],
        [0, sp.sin(phi), sp.cos(phi)]
    ])

def Rz(phi):
    """Rotation matrix around z-axis"""
    return sp.Matrix([
        [sp.cos(phi), -sp.sin(phi), 0],
        [sp.sin(phi), sp.cos(phi), 0],
        [0, 0, 1]
    ])

def DH2A(b, theta, a, alpha):
    """Denavit-Hartenberg transformation matrix"""
    ps = sp.Matrix([[0, 0, 0, 1]])
    Tb = sp.Matrix.vstack(
        sp.Matrix.hstack(sp.eye(3), sp.Matrix([[0], [0], [b]])),
        ps
    )
    Ttht = sp.Matrix.vstack(
        sp.Matrix.hstack(Rz(theta), sp.Matrix([[0], [0], [0]])),
        ps
    )
    Ta = sp.Matrix.vstack(
        sp.Matrix.hstack(sp.eye(3), sp.Matrix([[a], [0], [0]])),
        ps
    )
    Talp = sp.Matrix.vstack(
        sp.Matrix.hstack(Rx(alpha), sp.Matrix([[0], [0], [0]])),
        ps
    )
    return Tb * Ttht * Ta * Talp

def forward_kinematics(theta_vals, L):
    """Forward kinematics calculation"""
    # Assign symbolic vars
    theta1, theta2, theta3, theta4, theta5 = theta_syms
    L1, L2, L3, L4, L5 = length_syms
    
    A1 = DH2A(L1, theta1, 0, sp.pi/2)
    A1[0, 1] = 0
    A1[1, 1] = 0
    A1[2, 2] = 0
    
    A2 = DH2A(0, theta2 + sp.pi/2, L2, 0)
    A3 = DH2A(0, theta3, L3, 0)
    
    A4a = DH2A(0, theta4 + sp.pi/2, 0, sp.pi/2)
    A4a[0, 1] = 0
    A4a[1, 1] = 0
    A4a[2, 2] = 0
    
    A4b = DH2A(L4, 0, 0, 0)
    A5 = DH2A(L5, theta5, 0, 0)
    
    A4 = A4a * A4b
    T_sym = sp.simplify(A1 * A2 * A3 * A4 * A5)
    
    # Substitute values
    vars_list = list(theta_syms) + list(length_syms)
    vals_list = list(theta_vals) + list(L)
    T_sub = T_sym.subs(zip(vars_list, vals_list))
    
    # Check if all symbols are substituted
    if len(T_sub.free_symbols) > 0:
        remaining_symbols = ', '.join([str(sym) for sym in T_sub.free_symbols])
        raise ValueError(f"Substitution incomplete. Remaining symbols: {remaining_symbols}")
    
    return np.array(T_sub).astype(float)

def calculate_jacobian(theta_vals, L):
    """Calculate the Jacobian matrix"""
    delta = 1e-5  # small perturbation
    T0 = forward_kinematics(theta_vals, L)
    p0 = T0[0:3, 3]
    J = np.zeros((3, len(theta_vals)))
    
    for i in range(len(theta_vals)):
        theta_temp = theta_vals.copy()
        theta_temp[i] = theta_temp[i] + delta
        T_new = forward_kinematics(theta_temp, L)
        p_new = T_new[0:3, 3]
        J[:, i] = (p_new - p0) / delta
    
    return J

def inverse_kinematics(T_goal, theta_init, L):
    """Inverse kinematics using Jacobian method"""
    theta_vals = theta_init.copy()
    max_iter = 100
    tol = 1e-4
    
    for i in range(max_iter):
        T_curr = forward_kinematics(theta_vals, L)
        p_curr = T_curr[0:3, 3]
        p_goal = T_goal[0:3, 3]
        error = p_goal - p_curr
        
        if np.linalg.norm(error) < tol:
            print(f"Converged in {i+1} iterations.")
            return theta_vals
        
        J = calculate_jacobian(theta_vals, L)
        delta_theta = np.linalg.pinv(J) @ error
        theta_vals = theta_vals + delta_theta
    
    print("Warning: IK did not converge within max iterations.")
    return theta_vals

# Main script
if __name__ == "__main__":
    # Input link lengths
    L = np.array([25.929, 81.379, 83.009, 77, 106])
    
    # Initial joint angles guess (in radians)
    theta_init = np.array([0, 0, 0, -np.pi/2, np.pi/2])
    
    # Desired end-effector position (x, y, z)
    T_goal = np.eye(4)
    T_goal[0:3, 3] = np.array([160.0, 0, 35.0])
    
    # Run inverse kinematics
    theta_solution = inverse_kinematics(T_goal, theta_init, L)
    
    print("Final joint angles (in degrees):")
    joint_angles_deg = np.degrees(theta_solution)
    for i, angle in enumerate(joint_angles_deg):
        print(f"Joint {i+1}: {angle:.2f}°")

    # Convert to servo angles
    servo_angles = [
        joint_angles_deg[0] + 90,
        joint_angles_deg[1] + 90,
        joint_angles_deg[2] + 90,
        joint_angles_deg[3] + 90,
        joint_angles_deg[4]
    ]

    print("\nCorresponding servo angles (in degrees):")
    for i, angle in enumerate(servo_angles):
        print(f"Servo {i+1}: {angle:.2f}°")
    
    print("Final end-effector position:")
    T_final = forward_kinematics(theta_solution, L)
    print(T_final[0:3, 3])