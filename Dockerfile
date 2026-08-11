FROM ros:melodic

ENV USERNAME agilicious
ENV HOME /home/$USERNAME

# nvidia-container-runtime. Adds support to Nvidia drivers inside the container.
# for this to work, you need to install nvidia-docker2 in your host machine.
# More info: http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN useradd -m $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
        usermod -aG sudo $USERNAME && \
        mkdir -p /etc/sudoers.d && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME && \
        # Replace 1000 with your user/group id
        usermod  --uid 1000 $USERNAME && \
  groupmod --gid 1000 $USERNAME

USER agilicious
WORKDIR /home/${USERNAME}

RUN sudo apt-get update

  # GCC-9
RUN sudo apt-get install -y software-properties-common
RUN sudo add-apt-repository ppa:ubuntu-toolchain-r/test
RUN sudo apt-get update
RUN sudo apt-get install -y gcc-9 g++-9

  # Set gcc-9 default GCC compiler
RUN sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 60 --slave /usr/bin/g++ g++ /usr/bin/g++-9

  # Install CLANG
RUN sudo apt-get install -y clang-10

  # Set CLANG the default compiler
ENV CC /usr/bin/clang-10
ENV CXX /usr/bin/clang++-10

  # Python and git:
RUN sudo apt-get install -y python python-pip python3-pip git
  # Python packages
RUN sudo pip install catkin-tools scipy

  # Libraries
RUN sudo apt-get install -y libyaml-cpp-dev libeigen3-dev libgoogle-glog-dev ccache tmux  net-tools iputils-ping nano wget usbutils htop gdb psmisc screen

  # nuttx dependencies
RUN sudo apt-get install -y automake bison build-essential flex gcc-arm-none-eabi gperf git libncurses5-dev libtool libusb-dev libusb-1.0.0-dev pkg-config dfu-util
RUN mkdir -p ~/nuttx
RUN cd ~/nuttx && git clone https://bitbucket.org/nuttx/tools && cd tools/kconfig-frontends/ && ./configure && make && sudo make install && sudo ldconfig

RUN sudo apt-get update

  # ROS dependencies:
RUN sudo apt-get install -y \
  ros-${ROS_DISTRO}-mav-msgs \
  ros-${ROS_DISTRO}-octomap-msgs \
  ros-${ROS_DISTRO}-octomap-ros \
  ros-${ROS_DISTRO}-gazebo-plugins \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-rqt \
  ros-${ROS_DISTRO}-rviz \
  ros-${ROS_DISTRO}-plotjuggler-ros

  # Install betaflight:
RUN sudo mkdir /usr/share/desktop-directories/
RUN sudo apt-get install -y \
        gpg \
        gpg-agent \
        gpgconf \
        gpgv \
        libcanberra-gtk-module \
        libcanberra-gtk3-module \
        libgconf-2-4 \
        libgtk-3-0 \
        libnss3 \
        libx11-xcb1 \
        libxss1 \
        libxtst6 \
        dbus \
        xdg-utils

RUN mkdir -p ~/betaflight_download/ && cd ~/betaflight_download && wget https://github.com/betaflight/betaflight-configurator/releases/download/10.7.0/betaflight-configurator_10.7.0_amd64.deb
RUN cd ~/betaflight_download && sudo dpkg -i betaflight-configurator_10.7.0_amd64.deb
RUN rm -rf ~/betaflight_download

  # Install gazebo:
RUN sudo apt-get install -y gazebo9 libgazebo9-dev

  # Install and configure perf
RUN sudo apt-get update
RUN sudo apt-get install -y linux-tools-generic
RUN cd /usr/lib/linux-tools && cd `ls -1 | head -n1` && sudo rm -f /usr/bin/perf && sudo ln -s `pwd`/perf /usr/bin/perf

  # Create a catkin workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash"
RUN mkdir -p catkin_ws/src
RUN cd catkin_ws && catkin config --init --mkdirs --extend /opt/ros/$ROS_DISTRO --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release

  # Clone catkin_simple
RUN cd catkin_ws/src && git clone https://github.com/catkin/catkin_simple.git

  # Clone rotors_simulators from ASL
RUN cd catkin_ws/src && git clone https://github.com/ethz-asl/rotors_simulator.git

  # Clone mav_comm from ASL
RUN cd catkin_ws/src && git clone https://github.com/ethz-asl/mav_comm.git

# Clone eigen_catkin
RUN cd catkin_ws/src && git clone https://github.com/ethz-asl/eigen_catkin.git

  # Do catkin build of the packages that are already there
RUN cd catkin_ws && catkin build

  # Give permissions to use tty to user
RUN sudo usermod -a -G tty $USERNAME
RUN sudo usermod -a -G dialout $USERNAME

CMD "sudo /etc/init.d/dbus start"

  # Commands to build and run this (with GUI capabilities):
  # sudo docker build --tag "ros_agilicious:latest" .    # Run from dockerfile directory
  # ./launch_container.sh <your_agilicious_folder>   # launch the container with all GUI capabilities
