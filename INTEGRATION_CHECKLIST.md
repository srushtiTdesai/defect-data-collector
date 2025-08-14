# ROS 2 Humble Integration Checklist (Multi‑Machine, Pub/Sub + Services)

> Target distro: **Humble** (Ubuntu 22.04)  
> Network: Multi‑machine over LAN (DDS/RTPS)  
> Domain: **`ROS_DOMAIN_ID=7`**

Use this to verify **environment**, **nodes**, **networking**, **rosbag2**, and **cross‑machine Pub/Sub + Services** before integration day.

---

## 0) Quick Pass / TL;DR

```bash
# Per-shell
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0

ros2 --version
ros2 doctor --report

# Sanity: turtlesim
sudo apt install -y ros-humble-turtlesim ros-humble-demo-nodes-cpp
ros2 run turtlesim turtlesim_node             # Term A
ros2 run turtlesim turtle_teleop_key          # Term B
ros2 topic echo /turtle1/pose                 # Term C

# Cross-machine discovery:
ros2 multicast receive                        # on Machine A
ros2 multicast send                           # on Machine B (and vice versa)
```

---

## 1) System Setup

- [ ] **Ubuntu 22.04** fully updated.
- [ ] Install ROS 2 Humble (Desktop recommended):
  ```bash
  sudo apt update
  sudo apt install -y ros-humble-desktop python3-colcon-common-extensions       python3-rosdep python3-vcstool
  sudo rosdep init || true; rosdep update
  ```
- [ ] Add to **~/.bashrc**:
  ```bash
  # ROS 2 Humble
  source /opt/ros/humble/setup.bash
  # Team domain & network
  export ROS_DOMAIN_ID=7
  export ROS_LOCALHOST_ONLY=0
  # (Optional) lock RMW for consistency, default is Fast DDS
  # export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

  # Project overlay (adjust path)
  if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
  fi
  ```
- [ ] New shell:
  ```bash
  printenv | grep -E 'ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|RMW_IMPLEMENTATION'
  ros2 --version
  ros2 doctor --report
  ```

---

## 2) Workspace & Build

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 3) Local Verification (TurtleSim)

```bash
sudo apt install -y ros-humble-turtlesim
# Term A
ros2 run turtlesim turtlesim_node
# Term B (new shell)
ros2 topic list
ros2 topic echo /turtle1/pose
ros2 run turtlesim turtle_teleop_key
# One-off pub:
ros2 topic pub -1 /turtle1/cmd_vel geometry_msgs/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}"
# Service call:
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 255, g: 0, b: 0, width: 3, off: 0}"
# Spawn another turtle:
ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.5, y: 5.5, theta: 0.0, name: 'turtle2'}"
```

---

## 4) Standard Message Set (Team Default)

Use **standard** message types wherever possible.

- **Joints:** `sensor_msgs/msg/JointState`
- **Poses (multi):** `geometry_msgs/msg/PoseArray`
- **Single pose / stamped:** `geometry_msgs/msg/Pose`, `geometry_msgs/msg/PoseStamped`
- **Images:** `sensor_msgs/msg/Image` (optionally `image_transport` for compression)
- **Depth/3D:** `sensor_msgs/msg/PointCloud2`
- **Strings/Logs/IDs:** `std_msgs/msg/String`, `std_msgs/msg/Bool`, `std_msgs/msg/Int32`
- **TF:** `tf2_msgs/msg/TFMessage` via `tf2_ros`

> If you believe you need a **custom** message: first propose a mapping to a standard message (or composition of them). Only add custom types with team approval.

---

## 5) Topic & Service Registry (Fill‑In)

> **Please complete this table for your node(s) in your PR/README.**

### 5.1 Topics (Published/Subscribed)

| Name | Direction | Type | QoS (reliability/durability/history/depth) | Rate (Hz) | Notes |
|---|---|---|---|---|---|
| `/robot/joint_states` | Pub | `sensor_msgs/JointState` | reliable/volatile/keep_last/50 | 50 | Robot source of truth |
| `/poses/targets` | Pub | `geometry_msgs/PoseArray` | reliable/volatile/keep_last/10 | 5 | Planner outputs |
| `/camera/color/image_raw` | Pub | `sensor_msgs/Image` | best_effort/volatile/keep_last/5 | 15–30 | Consider image_transport |
| `/cloud/points` | Pub | `sensor_msgs/PointCloud2` | best_effort/volatile/keep_last/5 | 5–15 | High bandwidth |
| `/planner/goal` | Sub | `geometry_msgs/PoseStamped` | reliable/volatile/keep_last/10 | n/a | Planner goal input |
| `/status` | Pub | `std_msgs/String` | reliable/transient_local/keep_last/1 | on change | Latched status |

*(Add rows as needed.)*

### 5.2 Services (Provided/Called)

| Name | Role | Type | Notes |
|---|---|---|---|
| `/perception/reset` | Server | `std_srvs/srv/Trigger` | Reset node state |
| `/planner/plan_path` | Server | `nav_msgs/srv/GetPlan` | (If applicable) |
| `/system/save_bag` | Client | `rosbag2_interfaces/srv/RecordTopics` | Optional if using service‑based record |

---

## 6) QoS Guidance (Defaults to start)

- **Telemetry / commands / joint states / planning:** `reliable`, `volatile`, `keep_last`, `depth=10–50`.
- **High‑rate sensors (image/pointcloud):** `best_effort`, `volatile`, `keep_last`, `depth=5–10` to reduce backpressure.
- **Status / config / static info:** consider `transient_local` durability (latched‑like delivery to late joiners).
- Inspect live QoS:
  ```bash
  ros2 topic info /camera/color/image_raw --verbose
  ```

---

## 7) rosbag2 (MCAP) — Record & Replay

```bash
# Record a focused set (edit topics as needed)
ros2 bag record --storage mcap -o integration_smoke   /robot/joint_states /poses/targets /planner/goal   /camera/color/image_raw /cloud/points

# Inspect
ros2 bag info integration_smoke

# Replay
ros2 bag play integration_smoke
```

**Tips**
- Prefer **MCAP** storage for performance and compatibility.
- For very high‑rate topics, consider selective recording or compression.

---

## 8) Cross‑Machine Networking Checklist

**Match across all machines:**
- [ ] `ROS_DOMAIN_ID=7`
- [ ] `ROS_LOCALHOST_ONLY=0`
- [ ] Same ROS 2 distro (**Humble**) and (ideally) same `RMW_IMPLEMENTATION`
- [ ] NTP/chrony time sync enabled

**Firewall / Switch / VLAN:**
- [ ] Multicast **enabled** on LAN/VLAN
- [ ] Allow **UDP** for DDS/RTPS discovery/data (e.g., Fast DDS typical ranges around 7400–7600/udp; adapt to your policy)

**Discovery sanity:**
```bash
# Machine A
ros2 multicast receive
# Machine B
ros2 multicast send
# Expect packets on A; swap roles and re-check.
```

**Talker/Listener over LAN:**
```bash
sudo apt install -y ros-humble-demo-nodes-cpp
# Machine A
export ROS_DOMAIN_ID=7; export ROS_LOCALHOST_ONLY=0
ros2 run demo_nodes_cpp talker
# Machine B
export ROS_DOMAIN_ID=7; export ROS_LOCALHOST_ONLY=0
ros2 run demo_nodes_cpp listener
```

**Service call over LAN (example):**
- Run `turtlesim_node` on **Machine A**
- From **Machine B**:
  ```bash
  ros2 service list | grep turtle
  ros2 service call /spawn turtlesim/srv/Spawn "{x: 8.0, y: 8.0, theta: 0.0, name: 'remote_turtle'}"
  ```

---

## 9) Node Handoff Requirements (per contributor)

Include in your PR/README:

1. **How to build & run** (commands, dependencies, parameters).
2. **Published topics**: name, type, QoS, expected rate.
3. **Subscribed topics**: name, type, QoS.
4. **Services**: provided and consumed (name, type).
5. **Parameters** + defaults (YAML).
6. **Launch file** starting your node with params.
7. **Sample rosbag** (MCAP) for validation.
8. **Bandwidth profile** (rough estimates for images/pointclouds).

**Parameter YAML example**
```yaml
# config/<your_node>.yaml
your_node_name:
  ros__parameters:
    camera_frame: camera_color_optical_frame
    publish_rate_hz: 15
    save_debug_images: false
```

**Launch example**
```python
# launch/your_node.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_pkg',
            executable='your_node',
            name='your_node',
            output='screen',
            parameters=['config/your_node.yaml'],
            remappings=[
                ('/camera/image_raw', '/camera/color/image_raw'),
            ],
        )
    ])
```

---

## 10) Pre‑Integration Self‑Check (Copy into your PR)

- [ ] `ros2 --version` = Humble; `ros2 doctor --report` shows no errors  
- [ ] `ROS_DOMAIN_ID=7`, `ROS_LOCALHOST_ONLY=0` exported  
- [ ] Local TurtleSim tests (topic echo, teleop, service call) pass  
- [ ] **Cross‑machine** `ros2 multicast send/receive` passes  
- [ ] **Cross‑machine** talker/listener passes  
- [ ] **Cross‑machine** service call passes  
- [ ] I can **record** & **replay** a bag (MCAP) of my topics  
- [ ] README contains **topics, services, QoS, params, launch**, sample bag  
- [ ] My node uses **standard messages** where possible (see §4)

---

## 11) Troubleshooting

- **No discovery / empty graph**  
  Check `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY=0`, disable VPN, confirm multicast not blocked, re‑run `ros2 multicast send/receive`.
- **Only localhost works**  
  Most likely multicast blocked on LAN/VLAN or host firewall (`ufw`). Coordinate with IT.
- **Topic visible but no data**  
  QoS mismatch. Check with `ros2 topic info --verbose`; align reliability/durability/depth.
- **High loss / lag**  
  Use `best_effort` for images/pointclouds; reduce rates/resolution; consider `image_transport` compression.
- **WSL2 specifics**  
  WSL2 uses NAT; cross‑machine multicast can be flaky. Validate Ubuntu‑to‑Ubuntu first. If using WSL2, keep both talker/listener in the same WSL instance for sanity before multi‑host tests.

---

## 12) Snippets (Common Standard Messages)

**Publish a `JointState`:**
```bash
ros2 topic pub /robot/joint_states sensor_msgs/JointState "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
  name: ['joint1','joint2'], position: [0.1, 0.2], velocity: [], effort: []
}" -r 10
```

**Publish a `PoseArray`:**
```bash
ros2 topic pub /poses/targets geometry_msgs/PoseArray "{
  header: {frame_id: 'map'},
  poses: [
    {position:{x:1,y:0,z:0}, orientation:{x:0,y:0,z:0,w:1}},
    {position:{x:2,y:0,z:0}, orientation:{x:0,y:0,z:0,w:1}}
  ]
}" -1
```

**Echo an Image (metadata only):**
```bash
ros2 topic echo /camera/color/image_raw --qos-profile sensor_data
```

**Show interface (type) definition:**
```bash
ros2 interface show sensor_msgs/msg/JointState
```

---

### Appendix A — Team Topic Name Conventions (Optional)
- Frames: `map`, `odom`, `base_link`, `camera_color_optical_frame`
- Namespaces per robot: `/robot_X/...`
- Use lowercase with underscores: `/planner/goal`, `/perception/reset`
