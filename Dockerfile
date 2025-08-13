FROM ros:humble-ros-core

# Base tools
RUN apt-get update && apt-get install -y     python3-pip python3-colcon-common-extensions python3-rosdep python3-yaml     && rm -rf /var/lib/apt/lists/*

# Parquet deps
RUN pip3 install --no-cache-dir pyarrow pandas

# Workspace
ENV ROS_DISTRO=humble
WORKDIR /root/ros2_ws

# Copy repo into src
COPY . src/defect_data_collector_repo

# Initialize rosdep & build
RUN rosdep init || true && rosdep update &&     . /opt/ros/$ROS_DISTRO/setup.sh &&     rosdep install --from-paths src -y --ignore-src &&     colcon build --symlink-install

# Default: launch sim + collector using the package launch file
SHELL ["/bin/bash","-lc"]
CMD source /opt/ros/$ROS_DISTRO/setup.bash &&     source /root/ros2_ws/install/setup.bash &&     ros2 launch defect_data_collector defect_collection.launch.py
