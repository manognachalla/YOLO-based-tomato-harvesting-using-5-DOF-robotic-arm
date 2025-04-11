# 🍅 Tomato Ripeness Detection & Harvesting with DOFBOT 🤖  

This repository contains the implementation of an AI-driven **tomato harvesting system** using a **DOFBOT 5DOF robotic arm**. The system detects tomatoes, classifies their ripeness, and picks them using inverse kinematics and optimized path planning.

## 🚀 Features  
- **YOLOv12-based Tomato Detection** 🏷️  
- **Ripeness Classification (Green, Semi-Ripe, Ripe)** 🌱🍅  
- **DOFBOT 5DOF Robotic Arm Control** 🤖  
- **Inverse Kinematics for Positioning** 🔄  
- **Basic Path Planning for Picking** 🛠️  
- **Deployment on Raspberry Pi for Real-time Execution** 🎥  

## 🛠️ System Workflow  
1. **YOLOv12 Model** detects and classifies tomatoes based on ripeness.  
2. **Camera captures images** and processes them in real-time.  
3. **Coordinates of ripe tomatoes** are extracted for robotic control.  
4. **Inverse Kinematics (IK)** calculates the arm's joint angles.  
5. **Path Planning Algorithm** determines the optimal picking sequence.  
6. **DOFBOT gripper picks the tomato** and moves it to a collection bin.  

## 📌 Requirements  
- Python 3.8+  
- OpenCV  
- PyTorch  
- YOLOv12  
- DOFBOT SDK  
- NumPy, SciPy (for IK calculations)  
- Flask (for UI, optional)  

## 🔧 Installation  
Clone the repository:  
```bash
git clone https://github.com/kaniska-m/Tomato-Ripeness-DOFBOT.git
cd Tomato-Ripeness-DOFBOT

