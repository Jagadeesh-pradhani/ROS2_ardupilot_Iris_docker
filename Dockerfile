#Created By Jagadeesh-P
#03-08-2024

FROM osrf/ros:humble-desktop

# Install necessary programs
RUN apt-get update \
    && apt-get install -y \
    nano \
    vim \
    git \
    curl \
    lsb-release \
    gnupg \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME

# Install gz-harmonic
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y gz-harmonic



####################################################################################################
# Set up ROS 2 workspace
USER $USERNAME
WORKDIR /home/$USERNAME
#COPY ardupilot /home/${USERNAME}/ardupilot
#COPY Micro-XRCE-DDS-Gen /home/${USERNAME}/Micro-XRCE-DDS-Gen


RUN sudo apt install default-jre \
    && sudo apt-get install gitk git-gui \
    && sudo apt-get install gcc-arm-none-eabi -y

RUN cd ~/ \
    && git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git

RUN cd ~/Micro-XRCE-DDS-Gen \
    && ./gradlew assemble

RUN cd ~/ \
    && git clone https://github.com/ArduPilot/ardupilot.git \
    && cd ardupilot \
    && git submodule update --init --recursive \
    && git submodule init \
    && git submodule update \
    && git status 

RUN cd ~/ardupilot \
    && ./waf distclean \
    && ./waf distclean \
    && ./waf configure --board MatekF405-Wing

RUN mkdir -p ~/ros2_ws/src \
    && cd ~/ros2_ws

COPY ros2.repos /home/${USERNAME}/ros2_ws/ros2.repos
COPY ros2_gz.repos /home/${USERNAME}/ros2_ws/ros2_gz.repos



RUN cd ~/ros2_ws/ \
    && vcs import --recursive --input  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src    \
    && sudo apt update \
    && rosdep update \
    && /bin/bash -c "source /opt/ros/humble/setup.bash"   \
    && rosdep install -y --from-paths src --ignore-src

#BUild
RUN cd ~/ros2_ws \
    && colcon build --packages-up-to ardupilot_dds_tests || true
RUN /bin/bash -c "source ~/ros2_ws/install/setup.bash"

RUN sudo rm /home/${USERNAME}/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh
COPY install-prereqs-ubuntu.sh /home/${USERNAME}/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh


RUN cd ~/ardupilot \
    && sudo apt-get install -y python3-pip \
    && sudo pip3 install future \
    && Tools/environment_install/install-prereqs-ubuntu.sh -y || true \
    && sudo apt-get install -y python3-pexpect \
    && ./waf clean \
    && ./waf configure --board sitl \
    && ./waf copter -v 

RUN cd ~/ardupilot/Tools/autotest \
    && sudo pip3 install MAVProxy


#ROS2 with SITL
RUN /bin/bash -c "source /opt/ros/humble/setup.bash"

#Build
RUN cd ~/ros2_ws/ \
    && colcon build --packages-up-to ardupilot_sitl || true
RUN /bin/bash -c "source ~/ros2_ws/install/setup.bash"

#ROS2 with SITL in GAZEBO
RUN cd ~/ros2_ws \
    && vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src || true  \
    && /bin/bash -c "source /opt/ros/humble/setup.bash" \
    && sudo apt update \
    && rosdep update \
    && rosdep install -y --from-paths src --ignore-src -r || true


#Build
RUN cd ~/ros2_ws \
    && colcon build --packages-up-to ardupilot_gz_bringup || true
RUN /bin/bash -c "source ~/ros2_ws/install/setup.bash"


####################################################################################################



# Copy the entrypoint and bashrc scripts so we have our container's environment set up correctly
COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc
COPY Ardupilot_ROS.sh /home/${USERNAME}/Ardupilot_ROS.sh


# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]
