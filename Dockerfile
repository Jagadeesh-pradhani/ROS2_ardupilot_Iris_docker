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
COPY ardupilot /home/${USERNAME}/ardupilot
COPY Micro-XRCE-DDS-Gen /home/${USERNAME}/Micro-XRCE-DDS-Gen


RUN sudo apt install default-jre \
    && sudo apt-get install gitk git-gui \
    && sudo apt-get install gcc-arm-none-eabi

RUN cd ~/Micro-XRCE-DDS-Gen \
    && ./gradlew assemble

RUN cd ~/ardupilot \
    && ./waf distclean \
    && ./waf distclean \
    && ./waf configure --board MatekF405-Wing \
    && ./waf plane


RUN mkdir -p ~/ros2_ws/src \
    && cd ~/ros2_ws

COPY ros2.repos /home/${USERNAME}/ros2_ws/ros2.repos
COPY ros2.repos /home/${USERNAME}/ros2_ws/ros2_gz.repos



RUN cd ~/ros2_ws/ \
    && vcs import --recursive --input  ./ros2.repos src \
    && sudo apt update \
    && rosdep update \
    && source /opt/ros/humble/setup.bash \
    && rosdep install -y --from-paths src --ignore-src

#BUild
RUN cd ~/ros2_ws \
    && colcon build --packages-up-to ardupilot_dds_tests 
RUN /bin/bash -c "source ~/ros2_ws/install/setup.bash"


RUN cd ~/ardupilot \
    && Tools/environment_install/install-prereqs-ubuntu.sh -y \
    && ./waf clean \
    && ./waf configure --board sitl \
    && ./waf copter -v 

RUN cd ~/ardupilot/Tools/autotest \
    && sudo pip3 install MAVProxy


#ROS2 with SITL
RUN /bin/bash -c "source /opt/ros/humble/setup.bash"

#Build
RUN cd ~/ros2_ws/ \
    && colcon build --packages-up-to ardupilot_sitl
RUN /bin/bash -c "source ~/ros2_ws/install/setup.bash"

#ROS2 with SITL in GAZEBO
RUN cd ~/ros2_ws \
    && vcs import --input ./ros2_gz.repos --recursive src \
    && source /opt/ros/humble/setup.bash \
    && sudo apt update \
    && rosdep update \
    && rosdep install -y --from-paths src --ignore-src -r


#Build
RUN cd ~/ros2_ws \
    && colcon build --packages-up-to ardupilot_gz_bringup
RUN /bin/bash -c "source ~/ros2_ws/install/setup.bash"


####################################################################################################



# Copy the entrypoint and bashrc scripts so we have our container's environment set up correctly
COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc
COPY Ardupilot_ROS.sh /home/${USERNAME}/Ardupilot_ROS.sh


# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]
