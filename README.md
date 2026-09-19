# ROMI Nav Robot (Kiyocore)

A differential/mecanum ROS 2 robot for **human-in-the-loop navigation data collection** — built for ROMI Lab's imitation-learning research (avoiding SLAM tracking failures at doorways/textureless areas).

At each step the robot presents **two candidate moves** — one from the Nav2 planner, one from an agent (stub, swappable for a trained model) — a **human chooses**, the robot drives a fixed ~8 cm step, and **every decision is logged** as imitation-learning training data.

---

## Hardware

| Part | Role |
|------|------|
| Raspberry Pi 4B (Ubuntu 22.04 + ROS 2 Humble) | On-board computer: sensors, motors, odometry |
| ESP32-WROOM | Motor controller (4× TT motors via 2× L298N), micro-ROS |
| ESP32-C3 Mini + analog joystick | Manual teleop controller (USB serial to laptop) |
| RPLIDAR A1 | 2D LiDAR (mapping + obstacle sensing) |
| MPU6050 | IMU (fused with laser odometry) |
| 4× mecanum wheels | Drive (skid-steer turn + strafe) |
| 4S 18650 pack + BMS | Power |
| Laptop (Ubuntu 22.04 + ROS 2 Humble) | Planner, recommendation node, RViz, joystick |

**Architecture:** Pi = sensors + motors + odometry. Laptop = planner + recommendation node + RViz + joystick. They talk over Wi-Fi (ROS 2 DDS, same `ROS_DOMAIN_ID`).

---

## Build From Scratch

### 1. Flash the Pi
- Raspberry Pi Imager → **Ubuntu Server 22.04 LTS (64-bit)** (NOT Desktop, NOT 24.04).
- In Imager settings: set hostname `kiyocore`, user `ubuntu`, enable SSH (password), enter Wi-Fi (**2.4 GHz**), country PK, locale Asia/Karachi.

### 2. Install ROS 2 Humble (on Pi AND laptop)
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-humble-ros-base ros-dev-tools python3-colcon-common-extensions   # laptop: ros-humble-desktop for RViz
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
source ~/.bashrc
```

### 3. Install dependencies
**Pi:**
```bash
sudo apt install -y ros-humble-rplidar-ros ros-humble-robot-localization ros-humble-slam-toolbox i2c-tools python3-smbus
# enable I2C: add 'dtparam=i2c_arm=on' to /boot/firmware/config.txt, then reboot
# micro-ROS agent: build from source in ~/microros_ws (see micro-ROS docs)
```
**Laptop:**
```bash
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup python3-serial
```

### 4. Get the code
```bash
git clone https://github.com/Zero-State-Logic/Romi-nav-robot.git
# Pi:    copy robot_ws + microros_ws + imu_node.py
# Laptop: copy romi_ws + joy_serial.py
cd ~/robot_ws && colcon build && source install/setup.bash      # Pi
cd ~/romi_ws  && colcon build && source install/setup.bash      # laptop
```

### 5. Flash the ESP32s
- **WROOM**: Arduino IDE → ESP32 Dev Module → add `micro_ros_arduino` (humble branch ZIP) → flash `firmware/wroom_mecanum.ino`.
- **C3 Mini**: Arduino IDE → ESP32C3 Dev Module → flash `firmware/c3_joystick.ino` (plain serial, no micro-ROS).

---

## Change / Set Up a New Wi-Fi Network

The Pi joins whatever Wi-Fi it's told to. To point it at a new network:

**If you can SSH into the Pi:**
```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```
Edit the `wifis` block:
```yaml
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "NEW_NETWORK_NAME":
          password: "NEW_PASSWORD"
```
(spaces only, no tabs) then:
```bash
sudo netplan apply
```

**If you CAN'T reach the Pi (no monitor):**
1. Power off, put the SD card in a computer.
2. Open the `system-boot` partition, edit `network-config`, set the new SSID/password.
3. Eject, reboot the Pi.

**Notes:** Pi Wi-Fi prefers **2.4 GHz**. Both Pi and laptop must be on the **same network** and same `ROS_DOMAIN_ID=0`. A phone hotspot works well (no client isolation). The Pi's IP may change per network — find it in the router / hotspot device list, then `ssh ubuntu@<IP>`.

---

## Running the Robot

> Power: Pi on wall power for mapping. WROOM + LiDAR → Pi. C3 joystick → laptop. Hotspot on, both devices connected.

### PI — Terminal 1 — LiDAR
```bash
sudo chmod 666 /dev/ttyUSB0
sudo stty -F /dev/ttyUSB0 115200 cs8 -cstopb -parenb
export ROS_DOMAIN_ID=0 && source /opt/ros/humble/setup.bash
ros2 run rplidar_ros rplidar_composition --ros-args \
  -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=115200 \
  -p frame_id:=laser -p angle_compensate:=true
```

### PI — Terminal 2 — Robot Bringup (motors + IMU + odometry)
```bash
sudo chmod 666 /dev/ttyACM0
export ROS_DOMAIN_ID=0 && source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
ros2 launch kiyocore_bringup bringup.launch.py
```

### PI — Terminal 3 — SLAM (only when mapping a room)
```bash
export ROS_DOMAIN_ID=0 && source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/home/ubuntu/robot_ws/install/kiyocore_bringup/share/kiyocore_bringup/config/slam.yaml
```
Drive one slow loop back to start, then save:
```bash
mkdir -p ~/robot_ws/maps
ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/maps/my_room
```

### LAPTOP — Terminal 1 — Joystick
```bash
sudo chmod 666 /dev/ttyACM0
export ROS_DOMAIN_ID=0 && source /opt/ros/humble/setup.bash
python3 ~/joy_serial.py
```
Joystick drives the robot. Button toggles strafe (mecanum).

### LAPTOP — Terminal 2 — Nav2 Planner
```bash
export ROS_DOMAIN_ID=0 && source /opt/ros/humble/setup.bash
source ~/romi_ws/install/setup.bash
ros2 launch romi_nav planner_only.launch.py
```

### LAPTOP — Terminal 3 — RViz
```bash
export ROS_DOMAIN_ID=0 && source /opt/ros/humble/setup.bash
rviz2
```
Fixed Frame = `odom`. Add `/scan`. Use **2D Goal Pose** to click a goal.

### LAPTOP — Terminal 4 — Recommendation Node
```bash
export ROS_DOMAIN_ID=0 && source /opt/ros/humble/setup.bash
source ~/romi_ws/install/setup.bash
ros2 run romi_nav recommendation_node
```
Click a goal in RViz. The node shows:
```
[1] Nav2 : <direction>
[2] Agent: <direction>
```
Choose: **1** = Nav2 drives one ~8 cm step | **2** = you drive that step with the joystick | **s** = skip | **q** = quit.
Every choice is logged to `~/romi_ws/data/decisions_<time>.jsonl`.

---

## The Data

`~/romi_ws/data/decisions_*.jsonl` (on the laptop) — each row: pose, goal, Nav2 suggestion, agent suggestion, chosen option. This is the imitation-learning dataset.

**Swap in a trained agent:** edit `agent_suggestion()` in `recommendation_node.py` — replace the stub with your model's output (same return: a target `(x, y)`).

---

## Shutdown
Ctrl+C all terminals. On the Pi: `sudo poweroff`, wait for the green LED to stop, then unplug.

---

*Built by Zero State Logic.*
