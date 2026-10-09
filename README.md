# Autonomous Military Drone Swarm
## Complete Installation & Running Guide

**Review 2 Prototype — ROS 2 Jazzy + Gazebo Harmonic + WSL2**

This guide explains how to install the required environment and run the two-drone Review 2 prototype from a clean Windows PC.

### Review 2 scope
- Gazebo simulation environment
- Two quadrotor drones
- Basic PID stabilization/hover control
- IMU and odometry sensing
- ROS 2 communication between Drone 1, Drone 2 and GCS
- Communication event monitoring
- Basic/theoretical communication latency representation

### Target stack
- Windows 10/11
- WSL2
- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Gazebo Harmonic
- Python / ROS 2
- colcon

> A public GitHub repository contains the source code, but WSL, Ubuntu, ROS 2, Gazebo and system dependencies must be installed on the new machine.

---

## 1. Install WSL2

Open **PowerShell as Administrator**:

```powershell
wsl --install
```

Restart Windows if requested.

Open **Ubuntu** from the Start menu and create a Linux username and password.

Verify from PowerShell:

```powershell
wsl -l -v
```

Ubuntu should show version `2`.

If necessary:

```powershell
wsl --set-version Ubuntu 2
```

---

## 2. Update Ubuntu and install basic tools

Inside Ubuntu:

```bash
sudo apt update
sudo apt upgrade -y

sudo apt install -y git curl wget unzip zip build-essential python3-pip python3-venv software-properties-common
```

---

## 3. Install ROS 2 Jazzy

ROS 2 Jazzy targets Ubuntu 24.04.

```bash
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository universe
```

Install the ROS apt source:

```bash
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')

curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
```

Install ROS 2 Desktop and development tools:

```bash
sudo apt install -y ros-jazzy-desktop
sudo apt install -y ros-dev-tools
```

Automatically source ROS in new terminals:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify:

```bash
printenv ROS_DISTRO
```

Expected:

```text
jazzy
```

---

## 4. Install rosdep

```bash
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update
```

---

## 5. Install Gazebo Harmonic integration

```bash
sudo apt install -y ros-jazzy-ros-gz
```

Verify:

```bash
gz sim --version
```

The project was developed/tested with Gazebo Harmonic.

---

## 6. Test ROS 2

Terminal 1:

```bash
ros2 run demo_nodes_cpp talker
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_py listener
```

The listener should receive messages.

---

## 7. Clone the GitHub project

Recommended:

```bash
cd ~
git clone https://github.com/SIMPLESOMEONE1202/drones_project.git drone_swarm_ws
cd ~/drone_swarm_ws
```

Expected source structure:

```text
src/
├── drone_msgs/
├── drone_description/
├── drone_gazebo/
├── drone_control/
├── drone_comms/
└── drone_bringup/
```

---

## 8. Install project dependencies

```bash
cd ~/drone_swarm_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

## 9. Build the project

```bash
cd ~/drone_swarm_ws
colcon build --symlink-install
source install/setup.bash
```

Verify:

```bash
ros2 pkg list | grep drone
```

Verify communication executables:

```bash
ros2 pkg executables drone_comms
```

Expected:

```text
drone_comms communication_node
drone_comms gcs_node
```

---

# 10. Run the complete Review 2 prototype

The tested startup uses multiple terminals.

### Terminal 1 — Gazebo server, world, bridges and drones

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_swarm_ws/install/setup.bash
ros2 launch drone_gazebo spawn_two_drones.launch.py
```

### Terminal 2 — Gazebo GUI

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_swarm_ws/install/setup.bash
gz sim -g --render-engine ogre2
```

### Terminal 3 — Drone 1 PID controller

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_swarm_ws/install/setup.bash
ros2 run drone_control pid_controller_node --ros-args -p drone_id:=drone_1
```

### Terminal 4 — Drone 2 PID controller

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_swarm_ws/install/setup.bash
ros2 run drone_control pid_controller_node --ros-args -p drone_id:=drone_2
```

### Terminal 5 — Drone 1 communication

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_swarm_ws/install/setup.bash
ros2 run drone_comms communication_node --ros-args -p drone_id:=drone_1
```

### Terminal 6 — Drone 2 communication

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_swarm_ws/install/setup.bash
ros2 run drone_comms communication_node --ros-args -p drone_id:=drone_2
```

### Terminal 7 — Ground Control Station

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_swarm_ws/install/setup.bash
ros2 run drone_comms gcs_node
```

### Terminal 8 — Communication event monitor

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_swarm_ws/install/setup.bash
ros2 topic echo /swarm/communication/events
```

### Terminal 9 — Drone 1 position

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_swarm_ws/install/setup.bash
ros2 topic echo /drone_1/odom --field pose.pose.position
```

### Terminal 10 — Drone 2 position

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_swarm_ws/install/setup.bash
ros2 topic echo /drone_2/odom --field pose.pose.position
```

---

## 11. Communication architecture

```text
                    GROUND CONTROL STATION
                           /                               /                            DRONE 1 <--> DRONE 2
```

Topics:

```text
/swarm/communication
/swarm/communication/events
```

`CommEvent` includes:

- `sender_id`
- `receiver_id`
- `event_type`
- `protocol_mode`
- `distance`
- `latency_ms`
- `battery_cost_percent`
- `success`

Review 2 intentionally keeps communication at the basic prototype level. Dynamic communication range, out-of-range behavior, reconnection and full communication-energy modeling are planned for Review 3.

---

## 12. Expected result

A successful run should provide:

- Gazebo border-style environment
- Ground and four watchtowers
- Drone 1 and Drone 2
- PID stabilization/hover
- IMU and odometry topics
- Drone-to-drone communication
- Drone-to-GCS communication
- Communication event monitoring
- Position telemetry

---

## 13. Repository structure

```text
drone_swarm_ws/
├── README.md
├── .gitignore
└── src/
    ├── drone_msgs/
    ├── drone_description/
    ├── drone_gazebo/
    ├── drone_control/
    ├── drone_comms/
    └── drone_bringup/
```

Generated locally after building:

```text
build/
install/
log/
```

These generated directories should normally not be committed to GitHub.

---

## 14. Troubleshooting

### `ros2: command not found`

```bash
source /opt/ros/jazzy/setup.bash
```

### Project packages not found

```bash
cd ~/drone_swarm_ws
source install/setup.bash
```

### Build failure

```bash
cd ~/drone_swarm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

### Gazebo unavailable

```bash
gz sim --version
```

If needed:

```bash
sudo apt install -y ros-jazzy-ros-gz
```

### Gazebo GUI problem under WSL2

Verify WSLg/graphics support and try:

```bash
gz sim -g --render-engine ogre2
```

### Python ROS executable not found

```bash
cd ~/drone_swarm_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 15. Quick start for an already configured machine

```bash
cd ~
git clone https://github.com/SIMPLESOMEONE1202/drones_project.git drone_swarm_ws
cd ~/drone_swarm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Then follow the 10-terminal startup sequence.

---

## 16. Portability note

A public GitHub repository contains the project's source code. It does not contain:

- WSL2
- Ubuntu
- ROS 2
- Gazebo
- system dependencies
- generated `build/`, `install/`, or `log/` directories

Therefore a new machine must install the required environment before cloning/building the project.

Target workflow:

**Windows → WSL2 → Ubuntu 24.04 → ROS 2 Jazzy → Gazebo Harmonic → Clone → rosdep → colcon build → Run**
