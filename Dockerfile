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

RUN vcs import src < src/elevation_mapping/elevation_mapping.repos && \
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