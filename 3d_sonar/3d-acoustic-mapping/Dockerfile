FROM ros:humble-ros-base

ARG USERNAME=rosdev
ARG UID=1000
ARG GID=$UID
# Install dependencies
RUN apt update -q \
    && apt upgrade -q -y \
    && apt install -y --no-install-recommends \
    software-properties-common \
    python3-pip \
    python3-dev \ 
    xauth \
    build-essential \
    cmake \
    git \
    libboost-all-dev \
    ros-humble-cv-bridge \
    netcat \
    iputils-ping \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-vision-msgs \
    ros-humble-sensor-msgs-py \
    libogre-1.12-dev \
    ros-humble-rviz2 \
    gfortran \
    libatlas-base-dev \
    libopenblas-dev \
    liblapack-dev \
    libfreetype6-dev \
    libpng-dev \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
# install python dependencies
RUN apt-get update && apt-get install -y python3-pip
COPY requirements.txt /tmp/
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Create a non-root user and set permissions
RUN groupadd -g $GID $USERNAME && \
    useradd -m -u $UID -g $GID -s /bin/bash $USERNAME && \
    apt-get update && apt-get install -y sudo && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Set the correct permissions for the workspace directory
ADD ros2_ws /home/$USERNAME/ros2_ws
RUN sudo chown -R $USERNAME:$USERNAME /home/$USERNAME/ros2_ws

##############################################################
# Make sure sudo commands has been executed before this line #
# Switch to the non-root user
USER $USERNAME

# If build, log and install directories are present, delete them
RUN sudo rm -rf /home/$USERNAME/ros2_ws/build /home/$USERNAME/ros2_ws/log /home/$USERNAME/ros2_ws/install

# Copy configuration files
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'source /home/'$USERNAME'/ros2_ws/install/setup.bash' >> /home/$USERNAME/.bashrc

# Build the ROS 2 workspace
WORKDIR /home/$USERNAME/ros2_ws

# Build the oculus_ros_services package first
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install"

# Source the workspace to include oculus_ros_services
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && source /home/$USERNAME/ros2_ws/install/setup.bash && rosdep update && rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install"

# Make middleware_profiles directory
RUN sudo mkdir -p /usr/local/share/middleware_profiles/

# Copy over the rtps_udp_profile.xml to the /usr/local/share/middleware_profiles/
COPY rtps_udp_profile.xml /usr/local/share/middleware_profiles/

CMD ["bash"]