FROM osrf/ros:noetic-desktop-full-focal

RUN apt-get update && apt-get install -y \
    ros-noetic-moveit \
    ros-noetic-catkin  \
    ros-noetic-trac-ik-kinematics-plugin \
    python3-catkin-tools  \
    python3-osrf-pycommon \
    python3-wstool \
    git \
    libqt5gui5

ENV QT_DEBUG_PLUGINS=1

# Setup catkin workspace
RUN mkdir -p /catkin_ws/src

WORKDIR /catkin_ws/src

# Setup moveit
RUN wstool init .  \
    && wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall  \
    && wstool update -t .

RUN git clone -b devel "https://github.com/eataiwo/cram.git" &&  \
    git clone -b cram_devel  "https://github.com/eataiwo/arbotix_ros.git"

RUN rosdep install -y --from-paths . --ignore-src --rosdistro noetic && rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws

# The following will install from Debian any package dependencies not already in your workspace:
RUN ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash"]

RUN catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release  \
    && catkin build &&

RUN ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash", "catkin_make", "ls /"]

#CMD ["roslaunch", "cram_v1_moveit", "cram_moveit.launch"]

COPY ./58-widowx.rules /etc/udev/rules.d
RUN udevadm control --reload-rules && udevadm trigger


# setup entrypoint
COPY ./ros-entrypoint.sh /

ENTRYPOINT ["/ros-entrypoint.sh"]
CMD ["bash"]





