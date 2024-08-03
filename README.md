# ROS2_ardupilot_Iris_docker

This repository contains a Docker setup for building and running ArduPilot SITL (Software In The Loop) with ROS2 integration. The setup is configured to continue the build process even if some steps produce errors, ensuring a more resilient build experience.

## Prerequisites

- Docker
  Install Docker: https://docs.docker.com/get-docker/

## Repository Structure

- `Dockerfile`: The Dockerfile containing the instructions to build the Docker image.
- `entrypoint.sh`: Entrypoint script.
- `install-prereqs-ubuntu.sh`: Requirements script for ardupilot.
- `bashrc`: Bashrc file for image.
- `inst.sh`: Instruction script for downloading without docker.
- `README.md`: This file.

## Getting Started
```
https://github.com/Jagadeesh-pradhani/ROS2_ardupilot_Iris_docker.git
cd ROS2_ardupilot_Iris_docker
```

### Cloning the Repository


