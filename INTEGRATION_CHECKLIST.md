# ROS 2 Humble Integration & User Guide (Multi‑Machine, Pub/Sub + Services)

> **Target distro:** ROS 2 Humble (Ubuntu 22.04)  
> **Network:** Multi‑machine over LAN using DDS/RTPS  
> **Domain:** `ROS_DOMAIN_ID=7`  
>  
> This guide helps you prepare your ROS 2 environment for integration with the rest of the team.  
> It explains not only **what** to do, but also **why** each step is important, so you understand how the pieces fit together.

---

## 0) Quick Pass / TL;DR — *Basic environment and network sanity check*

We start by verifying that you can run ROS 2 locally, see other machines on the network, and use a simple demo application (`turtlesim`).

```bash
# Source ROS 2 so the CLI tools work in this shell
source /opt/ros/humble/setup.bash

# Set our shared ROS 2 domain so nodes can talk across machines
export ROS_DOMAIN_ID=7

# Allow DDS discovery over the network, not just localhost
export ROS_LOCALHOST_ONLY=0

# Check that you have the correct ROS 2 version and that your environment is healthy
ros2 --version
ros2 doctor --report

# Install turtlesim for a lightweight test node
sudo apt install -y ros-humble-turtlesim ros-humble-demo-nodes-cpp

# Run turtlesim in one terminal (the 'server')
ros2 run turtlesim turtlesim_node

# Run teleop in another terminal (the 'client')
ros2 run turtlesim turtle_teleop_key

# Echo the turtle's position in a third terminal
ros2 topic echo /turtle1/pose

# Test network discovery between two machines
# On Machine A
ros2 multicast receive
# On Machine B
ros2 multicast send
# You should see messages appear on Machine A
```

---

## 1) System Setup — *Installing ROS 2 Humble*

ROS 2 must be installed and configured consistently across all team members. This ensures that the nodes will communicate without version or configuration mismatches.

```bash
# Update your package list
sudo apt update

# Install ROS 2 Humble Desktop (includes common tools like RViz)
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions     python3-rosdep python3-vcstool

# Initialize rosdep for dependency management
sudo rosdep init || true
rosdep update
```

Add this to your `~/.bashrc` so the environment variables are set automatically:

```bash
# ROS 2 Humble
source /opt/ros/humble/setup.bash

# Shared team domain
export ROS_DOMAIN_ID=7

# Allow cross-machine communication
export ROS_LOCALHOST_ONLY=0

# Optional: lock RMW to ensure consistent DDS implementation
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Source your workspace overlay if it exists
if [ -f ~/ros2_ws/install/setup.bash ]; then
  source ~/ros2_ws/install/setup.bash
fi
```

Reload your shell and confirm settings:

```bash
source ~/.bashrc
printenv | grep -E 'ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|RMW_IMPLEMENTATION'
ros2 --version
ros2 doctor --report
```

---

## 2) Workspace & Build — *Creating a clean build space*

We use a common workspace layout so it's easy for others to build and test your code.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 3) Local Verification (TurtleSim) — *Testing basic Pub/Sub and services locally*

`Turtlesim` is a simple 2D simulator that lets us quickly verify topics, services, and message passing.

```bash
sudo apt install -y ros-humble-turtlesim

# Terminal A: Start turtlesim
ros2 run turtlesim turtlesim_node

# Terminal B: See the available topics and echo the turtle pose
ros2 topic list
ros2 topic echo /turtle1/pose

# Control the turtle interactively
ros2 run turtlesim turtle_teleop_key

# Publish a one-off velocity message to move the turtle forward
ros2 topic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}"

# Call a service to change pen color
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 255, g: 0, b: 0, width: 3, off: 0}"

# Spawn another turtle via service
ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.5, y: 5.5, theta: 0.0, name: 'turtle2'}"
```

---

## 4) Standard Message Set — *Consistent data formats across the team*

To keep integration simple, we stick to standard ROS 2 message types unless absolutely necessary.

- **Joints:** `sensor_msgs/msg/JointState`
- **Multiple poses:** `geometry_msgs/msg/PoseArray`
- **Single pose:** `geometry_msgs/msg/Pose` or `geometry_msgs/msg/PoseStamped`
- **Images:** `sensor_msgs/msg/Image` (consider compression with `image_transport`)
- **3D point clouds:** `sensor_msgs/msg/PointCloud2`
- **Text/flags:** `std_msgs/msg/String`, `std_msgs/msg/Bool`, `std_msgs/msg/Int32`
- **Transforms:** `tf2_msgs/msg/TFMessage` via `tf2_ros`

If you think you need a custom message, first see if you can combine standard messages.

---

## 5) Topic & Service Registry — *Documenting your interface*

Each contributor must fill this table for their node so others know what to expect.

### Topics

| Name | Direction | Type | QoS | Rate (Hz) | Notes |
|------|-----------|------|-----|-----------|-------|
| `/robot/joint_states` | Pub | `sensor_msgs/JointState` | reliable/volatile/keep_last/50 | 50 | Robot state |
| `/poses/targets` | Pub | `geometry_msgs/PoseArray` | reliable/volatile/keep_last/10 | 5 | Planner output |
| `/camera/color/image_raw` | Pub | `sensor_msgs/Image` | best_effort/volatile/keep_last/5 | 15–30 | RGB camera |
| `/cloud/points` | Pub | `sensor_msgs/PointCloud2` | best_effort/volatile/keep_last/5 | 5–15 | 3D data |
| `/planner/goal` | Sub | `geometry_msgs/PoseStamped` | reliable/volatile/keep_last/10 | n/a | Planner input |
| `/status` | Pub | `std_msgs/String` | reliable/transient_local/keep_last/1 | on change | Status updates |

### Services

| Name | Role | Type | Notes |
|------|------|------|-------|
| `/perception/reset` | Server | `std_srvs/srv/Trigger` | Reset perception |
| `/planner/plan_path` | Server | `nav_msgs/srv/GetPlan` | Path planning |
| `/system/save_bag` | Client | `rosbag2_interfaces/srv/RecordTopics` | Trigger recording |

---

## 6) rosbag2 Basics — *Recording and replaying data*

We use MCAP storage for better performance and compatibility.

```bash
# Record selected topics
ros2 bag record --storage mcap -o integration_smoke   /robot/joint_states /poses/targets /planner/goal   /camera/color/image_raw /cloud/points

# Check bag contents
ros2 bag info integration_smoke

# Replay bag
ros2 bag play integration_smoke
```

---

## 7) Cross‑Machine Networking — *Verifying discovery and communication*

ROS 2 uses DDS, which relies on multicast for discovery.

**Checklist:**
- Same ROS 2 distro (Humble)
- Same `ROS_DOMAIN_ID`
- `ROS_LOCALHOST_ONLY=0`
- Firewall allows UDP multicast (default Fast DDS uses 7400–7600/udp)
- Clocks synced (ntp/chrony)

**Test discovery:**
```bash
# Machine A
ros2 multicast receive
# Machine B
ros2 multicast send
```

**Test Pub/Sub:**
```bash
# Machine A
ros2 run demo_nodes_cpp talker
# Machine B
ros2 run demo_nodes_cpp listener
```

**Test services:**
```bash
# Run turtlesim_node on Machine A
# From Machine B:
ros2 service list | grep turtle
ros2 service call /spawn turtlesim/srv/Spawn "{x: 8.0, y: 8.0, theta: 0.0, name: 'remote_turtle'}"
```

---

## 8) Pre‑Integration Self‑Check

- Humble installed and working (`ros2 --version`)
- Domain ID set to 7
- Local TurtleSim works
- Cross‑machine multicast works
- Cross‑machine talker/listener works
- Can record/replay MCAP bag
- Node interfaces documented

---

This file now acts as both **a setup checklist** and **a mini user guide** explaining why each step is needed.
