# ROS2_ardupilot_Iris_docker

This repository contains a Docker setup for building and running ArduPilot SITL (Software In The Loop) with ROS2 integration. The setup is configured to continue the build process even if some steps produce errors, ensuring a more resilient build experience.

## Prerequisites

- Docker <br>
  Install Docker: https://docs.docker.com/get-docker/

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
https://github.com/Jagadeesh-pradhani/ROS2_ardupilot_Iris_docker.git
cd ROS2_ardupilot_Iris_docker
```

### Building the Docker Image
```
docker build -t ros_humble .
```

### Running the Docker Container
```
docker run -it --user ros --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY ros_humble
```

## Specifications
- `ROS2 humble`
- `Gazebo Harmonic`

## Running Simulaiton

Use `Ardupilot_ros.ah` script file for commands and process.

### Terminal-1
```
cd ~/ros2_ws
source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

### Terminal-2
```
mavproxy.py --console --map --aircraft test --master=:14550
```

## Customizing the Build
The Dockerfile installs necessary dependencies, clones the ArduPilot repository, and runs the build process. It is designed to continue the build even if some steps fail.
Modify the Dockerfile for any changes.

## Troubleshooting
If you encounter issues during the build process, you can modify the Dockerfile to suit your needs. Ensure all necessary dependencies are installed and paths are correctly set.

## Contributing
Feel free to submit issues or pull requests if you have suggestions for improvements or find bugs.

## License
This project is licensed under the MIT License - see the LICENSE file for details.


