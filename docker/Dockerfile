FROM moveit/moveit:noetic-source

RUN apt-get update && apt-get install -y \
    ros-noetic-moveit \
    ros-noetic-catkin  \
    ros-noetic-trac-ik-kinematics-plugin \
    python3-catkin-tools  \
    python3-osrf-pycommon \
    python3-wstool \
    git \
    swig  # Needed for trac_ik

# Needed dependency for building trac-ik library from source
#RUN git clone https://github.com/stevengj/nlopt.git
#RUN mkdir -p /nlopt/build
#WORKDIR /nlopt/build
#RUN cmake ..
#RUN make
#RUN sudo make install
#WORKDIR /usr/local/lib
#RUN ln -s libnlopt.so libnlopt_cxx.so

## Setup catkin workspace
RUN mkdir -p /catkin_ws/src/cram/
COPY catkin_ws/src/cram_v1/ /catkin_ws/src/cram/

# Rename catkin workspace from moveit docker image
#RUN mkdir -p ~/cram/ 
#WORKDIR /

#RUN git clone -b cram_devel "https://github.com/eataiwo/arbotix_ros.git"
RUN cp -r -a ~/ws_moveit/src/. /catkin_ws/src/

WORKDIR /catkin_ws/src/

## Setup moveit
#RUN wstool init .  \
#    && wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall>
#    && wstool update -t .

#RUN git clone -b feature_simulation "https://github.com/eataiwo/cram.git" 
RUN git clone -b cram_devel "https://github.com/eataiwo/arbotix_ros.git"
#    git clone -b cram_devel  "https://github.com/eataiwo/trac_ik.git"

RUN rosdep install -y --from-paths . --ignore-src --rosdistro noetic && rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws

# The following will install from Debian any package dependencies not already in your workspace:
RUN ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash"]

RUN catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release \ 
    && catkin build

RUN ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash"]

# setup entrypoint
COPY docker/ros-entrypoint.sh /

ENTRYPOINT ["/ros-entrypoint.sh"]
CMD ["bash"]
