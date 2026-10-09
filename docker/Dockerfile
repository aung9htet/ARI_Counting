FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=42
RUN apt-get update && apt-get install -y --no-install-recommends \
    git build-essential python3-colcon-common-extensions python3-yaml python3-numpy \
    python3-opencv python3-pytest python3-jinja2 ffmpeg xvfb xauth mesa-utils \
    ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
    ros-humble-ros2-controllers ros-humble-moveit ros-humble-rmw-cyclonedds-cpp \
    ros-humble-realsense2-description ros-humble-launch-param-builder \
    ros-humble-ament-cmake-auto ros-humble-ament-cmake-python \
    ros-humble-cv-bridge ros-humble-rosbag2-storage-default-plugins \
    ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*
COPY dependencies.repos /opt/ari/dependencies.repos
COPY scripts/fetch_dependencies.py /opt/ari/fetch_dependencies.py
RUN python3 /opt/ari/fetch_dependencies.py /opt/ari_ws/src --manifest /opt/ari/dependencies.repos \
    && source /opt/ros/humble/setup.bash \
    && cd /opt/ari_ws \
    && colcon build --merge-install --packages-select launch_pal pal_urdf_utils ari_controller_configuration ari_description ari_moveit_config \
       --cmake-args -DBUILD_TESTING=OFF \
    && rm -rf build log src/*/.git
COPY src /opt/experiment_ws/src
RUN source /opt/ari_ws/install/setup.bash && cd /opt/experiment_ws \
    && colcon build --merge-install && rm -rf build log
COPY docker/entrypoint.sh /usr/local/bin/ari-entrypoint
COPY docker/run_application.py /usr/local/bin/ari-run-application
ARG HOST_UID=1000
ARG HOST_GID=1000
RUN (getent group ${HOST_GID} || groupadd -g ${HOST_GID} ari) \
    && (getent passwd ${HOST_UID} || useradd -u ${HOST_UID} -g ${HOST_GID} -m ari) \
    && chmod +x /usr/local/bin/ari-entrypoint && mkdir -p /workspace /data
WORKDIR /workspace
STOPSIGNAL SIGINT
ENTRYPOINT ["/usr/local/bin/ari-entrypoint"]
CMD ["ros2", "launch", "ari_counting_sim", "experiment.launch.py"]
