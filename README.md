# ROS2_ardupilot_Iris_docker

This repository contains a Docker setup for building and running ArduPilot SITL (Software In The Loop) with ROS2 integration. The setup is configured to continue the build process even if some steps produce errors, ensuring a more resilient build experience.

## Prerequisites

- Docker <br>
  Install Docker: https://docs.docker.com/get-docker/ <br>

  ```
  curl -fsSL https://get.docker.com -o get-docker.sh
  sudo sh ./get-docker.sh
  ```
  ```
  sudo groupadd docker
  sudo usermod -aG docker $USER
  systemctl is-enabled docker
  ```
  REBOOT the system

## Repository Structure

- `Dockerfile`: The Dockerfile containing the instructions to build the Docker image.
- `entrypoint.sh`: Entrypoint script.
- `install-prereqs-ubuntu.sh`: Requirements script for ardupilot.
- `bashrc`: Bashrc file for image.
- `inst.sh`: Instruction script for downloading without docker.
- `README.md`: This file.

## Getting Started

### Cloning the Repository
```
git clone https://github.com/Jagadeesh-pradhani/ROS2_ardupilot_Iris_docker.git
cd ROS2_ardupilot_Iris_docker
```

### Building the Docker Image
```
docker build -t ros_humble .
```

### Running the Docker Container
```
docker run -it --user ros --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY -v /dev/input:/dev/input --device-cgroup-rule='c 13:* rmw' ros_humble
```

## Specifications
- `ROS2 humble`
- `Gazebo Harmonic`

## Running Simulaiton

Use `Ardupilot_ros.ah` script file for commands and process. <br>
PC may crash while building 'ros2_gz'. <br>
close all other tabs except terminal and run following.
```
cd ~/ros2_ws/
colcon test --packages-select ardupilot_dds_tests
colcon build --packages-up-to ardupilot_sitl
colcon build --packages-up-to ardupilot_gz_bringup
source install/setup.bash
```

### Terminal-1
```
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_maze.launch.py
```
TO connect to Drone and to RC controller.
### Terminal-2
```
pip3 install pygame
mavproxy.py --console --map --aircraft test --master=:14550
module load joystick
```

Calibrate joy stick from QGC or Mission planner.




## Tips
Use `ctrl+d` to exit docker terminal. <br>
Use the commands shown above in `Running the Docker Container` only for the first time, next time onwards use following to open and close docker.<br>
This will open the loaded docker container, then continue with `Running Simulaiton`.
```
docker start  `docker ps -q -l` && docker attach `docker ps -q -l`
```


## Customizing the Build
The Dockerfile installs necessary dependencies, clones the ArduPilot repository, and runs the build process. It is designed to continue the build even if some steps fail.
Modify the Dockerfile for any changes.

## Troubleshooting
If theres a build error, comment out the lines containing `colcon build`.
If you encounter issues during the build process, you can modify the Dockerfile to suit your needs. Ensure all necessary dependencies are installed and paths are correctly set.

## Contributing
Feel free to submit issues or pull requests if you have suggestions for improvements or find bugs.

## License
This project is licensed under the MIT License - see the LICENSE file for details.


