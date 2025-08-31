# 🤖 RoArm QR Pick & Place

A **ROS 2 Humble**-based robotic arm system for **QR-code driven pick & place of momentos** 🎁.  
When a guest scans their QR code (via the `event_ui/` folder):  
- ✅ If the QR is in `allowed_guests.txt` → the arm picks and hands a momento.  
- 📝 The guest is logged in `scanned_guests.txt`.  
- ❌ If not in the list → access denied.  

---

## 📸 Simulation Preview  

<img width="709" height="638" alt="image" src="https://github.com/user-attachments/assets/b357d0e0-2d72-4806-9bed-74d217f5867b" />


---

## 📂 Project Layout  

```
roarm_ws/
│
├── src/                 # ROS 2 packages (modified RoArm packages)
├── event_ui/            # QR UI and helper scripts
├── allowed_guests.txt   # Allowed guest IDs (one per line)
├── scanned_guests.txt   # Guests already served
├── requirements.txt     # Python dependencies for event_ui
└── README.md
```

---

## ⚡ Quick Install (Ubuntu 22.04 — ROS 2 Humble)  

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install git
sudo apt install git -y

# Clone the repo into your workspace (important: final dot)
mkdir -p ~/roarm_ws
cd ~/roarm_ws
git clone https://github.com/harshilkp21/roarm_qr_pickplace.git .
```

---

## 📦 Add ROS2 Package Sources & Install Dependencies  

```bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade -y

# Install ROS 2 Humble and dependencies
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-dev-tools net-tools
sudo apt install -y ros-humble-moveit-*
sudo apt remove -y ros-humble-moveit-servo-*
sudo apt install -y ros-humble-generate-parameter-library
sudo apt install -y ros-humble-py-binding-tools
```
### 1️⃣ Install dependencies
```
rosdep install --from-paths src --ignore-src -r -y
```

---

## 🐍 Python Dependencies 

```bash
pip3 install -r requirements.txt
```
---

## 🏗️ Build — Package-by-Package (Recommended Order)  

Run these from workspace root (`~/roarm_ws`):  

```bash
cd ~/roarm_ws

colcon build --packages-select roarm_msgs
colcon build --packages-select moveit_servo
colcon build --packages-select rviz_marker_tools
colcon build --packages-select moveit_task_constructor_msgs
colcon build --packages-select moveit_task_constructor_core
colcon build --packages-select moveit_task_constructor_capabilities
colcon build --packages-select moveit_task_constructor_visualization
colcon build --packages-select roarm_moveit_cmd
colcon build --packages-select roarm_moveit_ikfast_plugins
colcon build --packages-select roarm_moveit_mtc_demo
colcon build --packages-select roarm_moveit_servo
colcon build --packages-select roarm_description roarm_driver roarm_moveit --symlink-install
```

After builds:  

```bash
source install/setup.bash
```

---

## 🤖 Set the Robot Model (RoArm M3)  

```bash
echo "export ROARM_MODEL=roarm_m3" >> ~/.bashrc
source ~/.bashrc
```

---


## ▶️ Running the Project

### 1️⃣ Launch the robot driver
```bash
ros2 run roarm_driver roarm_driver serial_port:=/dev/ttyUSB0
```

### 2️⃣ Launch the robot demo in MoveIt
```bash
ros2 launch roarm_moveit_mtc_demo demo.launch.py 
```

### 3️⃣ Start the Pick & Place Node
```bash
ros2 launch roarm_moveit_mtc_demo run.launch.py exe:=pick_place

```

### 4️⃣ Run the QR Scanner Node
```bash
ros2 run roarm_moveit_mtc_demo qr_detection.py 

```

## Now the UI part

### 1️⃣ Launch the Event UI (QR scanning app)
```bash
cd ~/roarm_ws/event_ui
python3 app.py
```
### 2️⃣ Start video streaming server
```bash
ros2 run web_video_server web_video_server
```


---

## 🛠️ Notes
- Make sure your robot is connected to `/dev/ttyUSB0`.  
- If needed, change permissions with:  
```
sudo chmod 666 /dev/ttyUSB0
```

---

## ✨ Features
- ✅ QR Code Detection 📷  
- ✅ Pick & Place Automation 🤖  
- ✅ ROS 2 & MoveIt Integration 🦾  

---

**Workflow**:  
Scan QR → `event_ui` checks `allowed_guests.txt` → if valid, triggers pick-and-place → updates `scanned_guests.txt` ✅  

---

## 📝 Notes & Tips  

- Always `source install/setup.bash` in new terminals OR do this,
```bash
echo "source ~/roarm_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
  
- To build only selected package:  

```bash
colcon build --packages-select <pkg_name>
```

- sometimes removing the build and installed packages resolves the errors
```bash
rm -rf build/<pkg_name> install/<pkg_name>
```

---
