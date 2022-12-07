FROM moveit/moveit:noetic-source

# Install known required libraries. Moveit, catkin, track_ik (swig), git etc
RUN apt-get update && apt-get install -y \
    ros-noetic-moveit \
    ros-noetic-catkin \
    swig \
    ros-noetic-trac-ik \
    ros-noetic-trac-ik-kinematics-plugin \
    python3-catkin-tools  \
    python3-osrf-pycommon \
    python3-wstool \
    git \
    && mkdir -p /catkin_ws/src/cram/ \
    && cp -r -a ~/ws_moveit/src/. /catkin_ws/src/

WORKDIR /catkin_ws/src/

RUN git clone -b cram_devel "https://github.com/eataiwo/arbotix_ros.git" # figure out how to git clone into a specificed path

WORKDIR /catkin_ws

RUN catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin build

COPY catkin_ws/src/cram_v1/ /catkin_ws/src/cram/

# Might be able to get rid of this catkin config as I already do it.
RUN catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin build cram_v1 \
    && rosdep install -y --from-paths . --ignore-src --rosdistro noetic  \
    && rm -rf /var/lib/apt/lists/*

RUN ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash"]

# setup entrypoint
COPY docker/ros-entrypoint.sh /

# Just for devel remove when in production/polished.
RUN ["/bin/bash", "-c", "echo source /opt/ros/${ROS_DISTRO}/setup.bash >> ~/.bashrc"]
RUN ["/bin/bash", "-c", "echo source /catkin_ws/devel/setup.bash >> ~/.bashrc"]

ENTRYPOINT ["/ros-entrypoint.sh"]
CMD ["bash"]
