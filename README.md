# Webots_World

This repository provides a customized and simplified Mars Yard environment adapted for use with Cyberboticcs's **Webots** Simualator and **ROS 2 Jazzy**.

---

### Key Contributions

- ✅ Created a custom world and added a rover for simulation  
- ✅ Added essential sensors for robot navigation including **Lidar**, **IMU**, and **Camera** sensors 

---

### Installation & Setup

1. **First**, use the following commmand to download **Webots**

```bash
sudo snap install webots
```

2. Create a workspace (if you don't already have one) and clone the repository in the **src** directory inside it
   ```bash
mkdir ros2_ws
cd ros2_ws && mkdir src
git clone git@github.com:Ibra1221/Webots_World.git
```

---

### How to Launch the Simulation

After building and sourcing, use the following command to run the simulation

```bash
ros2 launch my_package robot_launch.py
```

---

### Issues and Feedback

If you encounter any issues, have questions, or would like to suggest improvements, **please open an issue** on this GitHub repository. Your feedback is highly appreciated!

---

