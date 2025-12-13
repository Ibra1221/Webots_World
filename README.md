# Webots_World

This repository provides a customized and simplified Mars Yard environment adapted for use with Cyberbotics **Webots Simulator** and **ROS 2 Jazzy**.

---

### Key Contributions

* ✅ Created a custom world and added a rover for simulation
* ✅ Added essential sensors for robot navigation including **LiDAR**, **IMU**, and **Camera** sensors

---

### Installation & Setup

1. **First**, use the following command to download **Webots**

```bash
sudo snap install webots --classic
```

2. Create a workspace (if you don't already have one) and clone the repository in the **src** directory inside it

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Ibra1221/Webots_World.git
```

---

### How to Launch the Simulation

Use the following commands to run the simulation

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch my_package robot_launch.py
```

---

### Issues and Feedback

If you encounter any issues, have questions, or would like to suggest improvements, **please open an issue** on this GitHub repository. Your feedback is highly appreciated!

---
