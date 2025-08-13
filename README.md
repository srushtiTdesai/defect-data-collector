# defect_data_collector

ROS 2 (Humble) package to simulate and collect `geometry_msgs/PoseArray` from `/defect_gen/defect_pose`.
Collector writes **Parquet** (default) or **CSV** with batched, low-overhead I/O.

## Quick start (native)
```bash
# Ubuntu/WSL2
sudo apt update && sudo apt install -y python3-rosdep
cd ~/ros2_ws/src
git clone https://github.com/<you>/<repo>.git
cd ..
sudo rosdep init || true && rosdep update
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch defect_data_collector defect_collection.launch.py
```

## Docker (local build)
```bash
docker build -t defect-data-collector:latest .
mkdir -p ./defect_logs
# run collector-only with your params
docker run --rm -it --net=host   -v "$(pwd)/defect_logs:/root/defect_logs"   -v "$(pwd)/my_collector.yaml:/root/collector.yaml:ro"   defect-data-collector:latest   bash -lc 'source /opt/ros/$ROS_DISTRO/setup.bash; source /root/ros2_ws/install/setup.bash;             ros2 run defect_data_collector collector --ros-args --params-file /root/collector.yaml'
```

**Note:** In Docker, set `output_dir: "/root/defect_logs"` in YAML.

## Publish image to GHCR (optional)
```bash
echo "<YOUR_PAT>" | docker login ghcr.io -u <your_github_username> --password-stdin
docker build -t ghcr.io/<your_github_username>/defect-data-collector:0.1.0 .
docker tag ghcr.io/<your_github_username>/defect-data-collector:0.1.0 ghcr.io/<your_github_username>/defect-data-collector:latest
docker push ghcr.io/<your_github_username>/defect-data-collector:0.1.0
docker push ghcr.io/<your_github_username>/defect-data-collector:latest
```

On another PC:
```bash
docker pull ghcr.io/<your_github_username>/defect-data-collector:latest
mkdir -p defect_logs
docker run --rm -it --net=host   -v "$(pwd)/defect_logs:/root/defect_logs"   -v "$(pwd)/my_collector.yaml:/root/collector.yaml:ro"   ghcr.io/<your_github_username>/defect-data-collector:latest   bash -lc 'source /opt/ros/$ROS_DISTRO/setup.bash; source /root/ros2_ws/install/setup.bash;             ros2 run defect_data_collector collector --ros-args --params-file /root/collector.yaml'
```
