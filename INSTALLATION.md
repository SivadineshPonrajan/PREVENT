# 🤖 PREVENT - Installation instructions for Simulation

## 📋 Prerequisites
- 🐧 **Windows / Linux OS**
- 🎮 **GPU**: NVIDIA or AMD graphics card - **Highly recommended**
- 💾 **RAM**: 8GB+ recommended
- ⚙️ **`Docker desktop` software: If you are a windows user**

```
Apple Mac can't be used because of the silicon model based GPUs.
```
---

## 🚀 Quick Start Installation

Open the command prompt / terminal to run the following commands.

### 1️⃣ Clone the Repository
```bash
git clone https://github.com/SivadineshPonrajan/PREVENT.git

cd PREVENT
```

### 2️⃣ Build Docker Container

**If you are a windows user run the `Docker desktop` software before running these commands**

```bash
docker build -t tiryoh/ros2-desktop-vnc:humble .
```
⏱️ *This might take a few minutes...*

### 3️⃣ Launch the Container
```bash
docker run -p 6080:80 --gpus all --shm-size=32gb -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd)/:/code/ --privileged -it --name humble tiryoh/ros2-desktop-vnc:humble bash
```

### 4️⃣ Access Your Virtual Desktop
1. 🌐 Open your browser  
2. 🔗 Navigate to: **http://localhost:6080/**
3. 🔐 Login with:
   - **Username:** `ubuntu`  
   - **Password:** `ubuntu`

🎉 **Boom! You now have a Linux desktop in your browser!**

---

## 🔧 Setup ROS2 Environment

Open a terminal in your virtual desktop and run:

### 📦 Install Dependencies
```bash
sudo apt update && sudo apt install -y \
  ros-humble-turtlebot3* \
  ros-humble-turtlebot3-gazebo \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros \
  ros-humble-image-view \
  python3-numpy \
  python3-opencv \
  ros-humble-vision-opencv

sudo apt install python3-colcon-common-extensions
```

### 🐍 Fix Python Compatibility
```bash
sudo python3 -m pip uninstall -y numpy || true
sudo apt-get update
sudo apt-get install -y --reinstall python3-numpy
python3 -m pip install "numpy<2" --no-cache-dir
```

### 🏗️ Build the Project
```bash
cd ~/code/ros2_ws/
colcon build --symlink-install
```
🔨 *Compiling magic happening...*

### ⚙️ Environment Setup (IMPORTANT!)
**Run these commands in EVERY new terminal:**
```bash
source /opt/ros/humble/setup.bash
source /code/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
```

---

## 🚀 Launch the Simulation

Once everything is installed, open **3 terminals** and follow these steps:

### 🌍 Terminal 1: Start the World
```bash
# Setup environment first
source /opt/ros/humble/setup.bash
source /code/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

# Launch the simulation
ros2 launch prevent sim_tb3.launch.py
```
✨ **You'll see the robot spawned in a 3D Gazebo world!**

### 📷 Terminal 2: Robot Vision
```bash
# Setup environment first
source /opt/ros/humble/setup.bash
source /code/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

# View what the robot sees
ros2 run image_view image_view --ros-args --remap /image:=/camera/image_raw
```
👁️ **Watch the world through the robot's camera!**

### 🎮 Terminal 3: Drive the Robot manually (Optional)
```bash
# Setup environment first
source /opt/ros/humble/setup.bash
source /code/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

# Control the robot with keyboard
ros2 run turtlebot3_teleop teleop_keyboard
```
🕹️ **Use WASD keys to drive your robot around!**

---

