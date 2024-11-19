ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

WORKDIR /root/ws_elevation_mapping

COPY ./elevation_mapping src/elevation_mapping
COPY ./elevation_mapping_demos src/elevation_mapping_demos

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
# https://docs.docker.com/develop/develop-images/dockerfile_best-practices/#minimize-the-number-of-layers
RUN \
    # Update apt package list as previous containers clear the cache
    apt-get -q update && \
    apt-get -q -y upgrade --with-new-pkgs && \
    #
    # Install some base dependencies
    apt-get -q install --no-install-recommends -y \
    # Some basic requirements
    wget git sudo curl \
    # Preferred build tools
    clang clang-format-14 clang-tidy clang-tools

RUN if [ "${ROS_DISTRO}" = "humble" ]; then \
    wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null ; \
    apt-get update && apt-get install -y gz-garden ros-humble-ros-gzgarden; \
    else \
    apt-get update && apt-get install -y ros-${ROS_DISTRO}-ros-gz; \
    fi

RUN vcs import src < src/elevation_mapping/elevation_mapping.repos && \
    vcs import src < src/elevation_mapping_demos/elevation_mapping_demos.repos && \
    # Source ROS install
    . "/opt/ros/${ROS_DISTRO}/setup.sh" &&\
    # Install dependencies from rosdep
    apt-get -q update && \
    rosdep update && \
    DEBIAN_FRONTEND=noninteractive \
    rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    # rm -rf /var/lib/apt/lists/* && \
    # Build the workspace
    colcon build

RUN echo '. /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc && \
    echo '. /root/ws_elevation_mapping/install/setup.bash' >> /root/.bashrc && \
    echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/${ROS_DISTRO}/lib:/root/ws_elevation_mapping/install/leo_gz_plugins/lib' >> /root/.bashrc && \
    echo 'export GZ_SIM_RESOURCE_PATH=/opt/ros/${ROS_DISTRO}/share:$(ros2 pkg prefix leo_gz_worlds)/share/leo_gz_worlds/worlds:$(ros2 pkg prefix leo_gz_worlds)/share/leo_gz_worlds/models' >> /root/.bashrc;