FROM ruffsl/ros_armhf:kinetic

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
      ninja-build \
      python-rosdep \
      python-wstool && \
    rm -rf /var/lib/apt/lists/*

# setup catkin workspace
ENV CATKIN_WS=/home/pi/workspaces/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS

# Merge the cartographer_ros.rosinstall file and fetch code for dependencies.
RUN wstool init src && \
    wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall && \
    wstool update -t src


# Update apt-get because osrf image clears this cache. download deps
# Note that because we're building on top of kinetic-ci, there should not be any deps installed
# unless something has changed in the source code since the other container was made
# (they are triggered together so should only be one-build out of sync)
RUN apt-get -qq update && \
    apt-get -qq install -y \
        wget && \
    rosdep update && \
    rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    rm -rf /var/lib/apt/lists/*

# Replacing shell with bash for later docker build commands
RUN mv /bin/sh /bin/sh-old && \
    ln -s /bin/bash /bin/sh

    # Build repo
WORKDIR $CATKIN_WS
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8
RUN source "/opt/ros/$ROS_DISTRO/setup.bash" && \
    catkin_make_isolated --use-ninja
