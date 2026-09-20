FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg build-essential cmake git rsync patch pkg-config \
    ccache libeigen3-dev libyaml-cpp-dev libgoogle-glog-dev \
    cppzmq-dev libzmq3-dev \
    python3-colcon-common-extensions websockify python3-numpy \
    ros-jazzy-rmw-fastrtps-cpp libgl1-mesa-dri libglx-mesa0 xauth \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
       -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
       > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
       gz-harmonic libgz-sim8-dev libgz-plugin2-dev libgz-math7-dev \
       libgz-msgs10-dev libgz-transport13-dev \
    && rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /usr/local/bin/sitl-entrypoint
RUN chmod +x /usr/local/bin/sitl-entrypoint
WORKDIR /workspace/repo
ENTRYPOINT ["/usr/local/bin/sitl-entrypoint"]
