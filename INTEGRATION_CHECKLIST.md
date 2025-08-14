# ROS 2 Humble Integration & User Guide (Multi-Machine, Pub/Sub + Services)

> **Target distro:** ROS 2 Humble (Ubuntu 22.04)  
> **Network:** Multi-machine over LAN using DDS/RTPS  
> **Domain:** `ROS_DOMAIN_ID=7`  
>
> This guide helps you prepare your ROS 2 environment for team integration. It explains not only **what** to do, but also **why** each step matters.

---

## 0) Quick Pass / TL;DR — *Basic environment & network sanity check*

We verify you can run ROS 2 locally, see other machines on the LAN, and use a simple demo app (`turtlesim`).

```bash
# Source ROS 2 so the CLI works in this shell
source /opt/ros/humble/setup.bash

# Set our shared ROS 2 domain so nodes can talk across machines
export ROS_DOMAIN_ID=7

# Allow DDS discovery over the network, not just localhost
export ROS_LOCALHOST_ONLY=0

# Verify version and general health
ros2 --version
ros2 doctor --report

# Install a lightweight test node
sudo apt install -y ros-humble-turtlesim ros-humble-demo-nodes-cpp

# Run turtlesim (server)
ros2 run turtlesim turtlesim_node

# Run teleop (client)
ros2 run turtlesim turtle_teleop_key

# Echo the turtle's pose (telemetry)
ros2 topic echo /turtle1/pose

# Network discovery check between two machines
# On Machine A:
ros2 multicast receive
# On Machine B:
ros2 multicast send
```

---

## 1) System Setup — *Install & configure ROS 2 Humble consistently*

Consistency across the team avoids version/QoS/RMW mismatches.

```bash
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions     python3-rosdep python3-vcstool
sudo rosdep init || true
rosdep update
```

Add this to `~/.bashrc` so shells auto-configure:

```bash
# ROS 2 Humble
source /opt/ros/humble/setup.bash

# Shared team domain & LAN comms
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0

# Optional: lock the DDS impl for consistency (default is Fast DDS)
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Source your workspace overlay if present
if [ -f ~/ros2_ws/install/setup.bash ]; then
  source ~/ros2_ws/install/setup.bash
fi
```

Reload & verify:

```bash
source ~/.bashrc
printenv | grep -E 'ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|RMW_IMPLEMENTATION'
ros2 --version
ros2 doctor --report
```

---

## 2) Workspace & Build — *Clean, consistent layout*

A standard workspace makes your code easier to build and test.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 3) Local Verification (TurtleSim) — *Test Pub/Sub and services on one machine*

`turtlesim` quickly validates that topics and services work as expected.

```bash
sudo apt install -y ros-humble-turtlesim

# Terminal A: Start turtlesim
ros2 run turtlesim turtlesim_node

# Terminal B: Explore and observe
ros2 topic list
ros2 topic echo /turtle1/pose

# Terminal B: Drive the turtle
ros2 run turtlesim turtle_teleop_key

# Publish a one-off velocity command
ros2 topic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}"

# Call services (pen color)
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 255, g: 0, b: 0, width: 3, off: 0}"

# Spawn a second turtle
ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.5, y: 5.5, theta: 0.0, name: 'turtle2'}"
```

---

## 4) Standard Message Set — *Use common types to simplify integration*

Prefer standard ROS messages unless there’s a strong reason not to.

- **Joints:** `sensor_msgs/msg/JointState`  
- **Multiple poses:** `geometry_msgs/msg/PoseArray`  
- **Single pose:** `geometry_msgs/msg/Pose` or `geometry_msgs/msg/PoseStamped`  
- **Images:** `sensor_msgs/msg/Image` (consider `image_transport` compression)  
- **3D:** `sensor_msgs/msg/PointCloud2`  
- **Simple flags/text:** `std_msgs/msg/String`, `std_msgs/msg/Bool`, `std_msgs/msg/Int32`  
- **Transforms:** `tf2_msgs/msg/TFMessage` via `tf2_ros`  

*If you think you need a custom message, first try composing standard types.*

---

## 5) Topic & Service Registry — *Document your node’s interface*

Each contributor fills this table for their node.

### Topics

| Name | Direction | Type | QoS | Rate (Hz) | Notes |
|---|---|---|---|---|---|
| `/robot/joint_states` | Pub | `sensor_msgs/JointState` | reliable/volatile/keep_last/50 | 50 | Robot state |
| `/poses/targets` | Pub | `geometry_msgs/PoseArray` | reliable/volatile/keep_last/10 | 5 | Planner output |
| `/camera/color/image_raw` | Pub | `sensor_msgs/Image` | best_effort/volatile/keep_last/5 | 15–30 | RGB camera |
| `/cloud/points` | Pub | `sensor_msgs/PointCloud2` | best_effort/volatile/keep_last/5 | 5–15 | 3D data |
| `/planner/goal` | Sub | `geometry_msgs/PoseStamped` | reliable/volatile/keep_last/10 | n/a | Planner input |
| `/status` | Pub | `std_msgs/String` | reliable/transient_local/keep_last/1 | on change | Latched status |

### Services

| Name | Role | Type | Notes |
|---|---|---|---|
| `/perception/reset` | Server | `std_srvs/srv/Trigger` | Reset perception |
| `/planner/plan_path` | Server | `nav_msgs/srv/GetPlan` | Path planning |
| `/system/save_bag` | Client | `rosbag2_interfaces/srv/RecordTopics` | Trigger recording |

---

## 6) rosbag2 (MCAP) — *Turtlesim record & replay that retraces your path*

We demonstrate an end-to-end rosbag2 workflow with Turtlesim.  
**Key idea:** record `/turtle1/cmd_vel` (the **input**) while you teleop; replaying those commands drives a fresh turtlesim to **retrace** the same path. Recording `/turtle1/pose` alone won’t move anything—it’s just telemetry.

### 6.1 Start & drive
```bash
# Terminal A — run the simulator
ros2 run turtlesim turtlesim_node

# Terminal B — drive the turtle with arrow keys
ros2 run turtlesim turtle_teleop_key
```

### 6.2 Record the inputs (and pose for reference)
```bash
# Terminal C — record velocity commands + pose using MCAP storage
ros2 bag record --storage mcap -o turtle_run   /turtle1/cmd_vel /turtle1/pose
# Drive 10–20 seconds, then Ctrl-C to stop recording.
```

### 6.3 Inspect the bag
```bash
ros2 bag info turtle_run
```

### 6.4 Replay into a fresh turtlesim
```bash
# Stop the current turtlesim (Ctrl-C in Terminal A) to start clean.

# Terminal A — start a new simulator instance
ros2 run turtlesim turtlesim_node

# Terminal C — replay (publishes cmd_vel and pose)
ros2 bag play turtle_run
# The turtle should retrace your path.
```

**Tips**
- Remap if your topic names differ:
  ```bash
  ros2 bag play turtle_run --remap /turtle1/cmd_vel:=/my_turtle/cmd_vel
  ```
- Record only the control input to shrink bags:
  ```bash
  ros2 bag record --storage mcap -o turtle_run /turtle1/cmd_vel
  ```
- Change playback speed:
  ```bash
  ros2 bag play turtle_run --rate 0.5  # half speed
  ```

---

## 7) Cross-Machine Networking — *Discovery & communication checks*

ROS 2 uses DDS multicast for discovery. Align configs and open the right paths through your LAN/firewall.

**Checklist**
- Same ROS 2 distro (Humble)
- `ROS_DOMAIN_ID=7`
- `ROS_LOCALHOST_ONLY=0`
- Multicast allowed on LAN/VLAN; open typical Fast DDS UDP ranges (e.g., 7400–7600/udp per policy)
- Clocks synced (ntp/chrony)

**Discovery**
```bash
# Machine A
ros2 multicast receive
# Machine B
ros2 multicast send
```

**Pub/Sub**
```bash
# Machine A
ros2 run demo_nodes_cpp talker
# Machine B
ros2 run demo_nodes_cpp listener
```

**Services**
```bash
# Run turtlesim_node on Machine A
# From Machine B:
ros2 service list | grep turtle
ros2 service call /spawn turtlesim/srv/Spawn "{x: 8.0, y: 8.0, theta: 0.0, name: 'remote_turtle'}"
```

---

## 8) Pre-Integration Self-Check

- Humble installed & healthy (`ros2 --version`, `ros2 doctor --report`)  
- `ROS_DOMAIN_ID=7`, `ROS_LOCALHOST_ONLY=0` set  
- Local Turtlesim: topic echo, teleop, service call all pass  
- Cross-machine `ros2 multicast` send/receive pass  
- Cross-machine talker/listener pass  
- **Turtlesim rosbag2 replay retraces path**  
- You can record/replay MCAP for your node’s topics  
- Your node’s README documents topics, services, QoS, params, launch

---

## 9) Appendix — QoS quick guidance & handy commands

**QoS defaults to start**
- Commands / joint states / planner I/O: `reliable`, `volatile`, `keep_last`, depth 10–50  
- High-rate sensors (images/pointclouds): `best_effort`, `volatile`, `keep_last`, depth 5–10  
- Status/config/static info: consider `transient_local` (latched-like to late joiners)

**Inspect QoS & types**
```bash
ros2 topic info /camera/color/image_raw --verbose
ros2 interface show sensor_msgs/msg/JointState
```

**Common pitfalls**
- Nothing shows up → check `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY=0`, VPNs, firewalls, multicast.  
- Topic visible but no data → QoS mismatch; align reliability/durability/depth.  
- Only localhost works → multicast blocked on LAN/VLAN; coordinate with IT.  
- WSL2 quirks → NAT can break multicast; validate Ubuntu↔Ubuntu first.
