# syntax=docker/dockerfile:1

ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

# Build args (can be overridden by docker compose build args)
ARG ROS_DISTRO=humble
ARG USERNAME=rosdev
ARG UID=1000
ARG GID=1000

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    USERNAME=${USERNAME} \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

SHELL ["/bin/bash", "-c"]

# Base tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    vim \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    ros-${ROS_DISTRO}-pcl-msgs \
    ros-${ROS_DISTRO}-builtin-interfaces \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-sensor-msgs-py \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-std-srvs \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-rclpy \
    python3-setuptools \
    libopencv-dev \
    sudo \
    && rm -rf /var/lib/apt/lists/*
## Jupiter stuff 
RUN python3 -m pip install --no-cache-dir \
    jupyter \
    jupyterlab \
    ipykernel


# Create/ensure single dev user "rosdev" without failing if GID already exists
RUN set -eux; \
    # If UID already exists, reuse that user (rename to rosdev if needed)
    if getent passwd "${UID}" >/dev/null; then \
        EXISTING_USER="$(getent passwd "${UID}" | cut -d: -f1)"; \
        echo "UID ${UID} already belongs to ${EXISTING_USER}"; \
        if [ "${EXISTING_USER}" != "${USERNAME}" ]; then \
            usermod -l "${USERNAME}" "${EXISTING_USER}"; \
            usermod -d "/home/${USERNAME}" -m "${USERNAME}"; \
        fi; \
    else \
        # Ensure group with desired GID exists (create if missing)
        if ! getent group "${GID}" >/dev/null; then \
            groupadd --gid "${GID}" "${USERNAME}"; \
        fi; \
        # Create the user
        useradd --uid "${UID}" --gid "${GID}" -m -s /bin/bash "${USERNAME}"; \
    fi; \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Create Jupyter kernel with ROS environment
RUN mkdir -p /home/${USERNAME}/.local/share/jupyter/kernels/ros2 && \
printf '{\n  "display_name": "Python 3 (ROS2)",\n  "language": "python",\n  "argv": [\n    "bash",\n    "-c",\n    "source /opt/ros/%s/setup.bash && source /home/%s/ros2_ws/install/setup.bash 2>/dev/null || true && exec python3 -m ipykernel_launcher -f {connection_file}"\n  ]\n}\n' "${ROS_DISTRO}" "${USERNAME}" > /home/${USERNAME}/.local/share/jupyter/kernels/ros2/kernel.json && \
chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/.local/share/jupyter/kernels/ros2
# Workspace
USER ${USERNAME}
WORKDIR /home/${USERNAME}/ros2_ws
RUN mkdir -p src tests reports

# Entrypoint
USER root
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
USER ${USERNAME}
ENTRYPOINT ["/ros_entrypoint.sh"]

