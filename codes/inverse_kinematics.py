from roboticstoolbox import DHRobot, RevoluteDH
import numpy as np

# Define your robot using DH parameters
robot = DHRobot([
    RevoluteDH(d=0.025, a=0,      alpha=np.pi/2),     # Joint 1
    RevoluteDH(d=0,     a=0.0813, alpha=0),           # Joint 2
    RevoluteDH(d=0,     a=0.083,  alpha=0),           # Joint 3
    RevoluteDH(d=0.077, a=0,      alpha=np.pi/2),     # Joint 4 (combined 4a and 4b)
    RevoluteDH(d=0.106, a=0,      alpha=0)            # Joint 5 + end-effector offset
], name='My5DOFRobot')

# Desired end-effector pose
from spatialmath import SE3
T = SE3(0.150,0.0565,-0.034) * SE3.RPY([30, 45, 60], order='xyz', unit='deg')

# Solve IK using numerical method
sol = robot.ikine_LM(T, mask=[1, 1, 1, 0, 0, 0])  # only xyz

# Check solution and output
if sol.success:
    print("Joint angles (degrees):", np.degrees(sol.q))
else:
    print("IK solution not found.")