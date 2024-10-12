## Navigation in Ardupilot ROS2-gazebo drone

Let’s quickly understand what Nav2 is and why we need it. <br>

And before we get started, let’s just be clear on the terms we are going to use. When I write about Navigation, Navigation 2, Nav2, well this is all the same, I am writing about the Navigation 2 stack in ROS2 (also if I write ROS I also mean ROS2). <br>

So, why do we need Nav2 in the first place? Well, as you probably already know, ROS is a great framework that allows you to develop new robotics applications in no time. You get a lot of “plumbing” already done for you, a huge set of plug & play libraries, a community to help you, etc. <br>

Now, how do you go from knowing the ROS basics, to successfully implementing navigation for a mobile robot? This is not trivial at all, and fortunately, the Navigation 2 stack was developed just for that. The “stack” is a collection of packages that have been developed with one goal in mind. <br>

And what is this goal? The main goal is to make a robot move from point A to point B, in a safe way. So, basically, to find a path that allows the robot to reach the destination, while making sure that it will not collide with other robots, with people, and obstacles. <br>

## Steps
To achieve this, we will do a 2 step process:
1. We will create a map of the environment using `Cartographer`.
2. We will make the robot navigate using this map – with the Navigation functionalities and tools.

## 1. Cartographer running with LiDAR on copter
This simulation has an Iris copter equipped with a 3D LiDAR in a maze world. To launch rviz and gazebo, run:
```
ros2 launch ardupilot_ros cartographer.launch.py
```

## 2. Obstacle avoidance using Cartographer and Nav2
Using the same simulation as before, the nav2 node can be launched to control the copter once it is in the air.
Launch the simulation:
```
ros2 launch ardupilot_ros navigation.launch.py
```
You may now navigate while mapping using the Nav2 Goal tool in RVIZ!

## DDS/micro-Ros
Ardupilot contains the DDS Client library, which can run as SITL. Then, the DDS application runs a ROS 2 node, an eProsima Integration Service, and the MicroXRCE Agent. The two systems communicate over serial or UDP. <br>
**AP_DDS** is a library within ArduPilot that uses the Data Distribution Service (DDS) protocol for communication. It enables seamless data exchange between ArduPilot (the flight controller) and external systems like ROS2. Through this library, ArduPilot can publish critical telemetry data (e.g., IMU, GPS) and receive commands or other messages over DDS, integrating with ROS2 by using micro-ROS for resource-efficient communication. Topics are published in ROS2, facilitating control and sensor interaction with drones.
```mermaid
---
title: UDP Loopback
---
graph LR

  subgraph Linux Computer

    subgraph Ardupilot SITL
      veh[sim_vehicle.py] <--> xrceClient[EProsima Micro XRCE DDS Client]
      xrceClient <--> port1[udp:2019]
    end

    subgraph DDS Application
      ros[ROS 2 Node] <--> agent[Micro ROS Agent]
      agent <-->port1[udp:2019]
    end

    loopback

  end
```
Once the simulation runs You should be able to see the agent here and view the data output.

```bash
$ ros2 node list
/ardupilot_dds
```

```bash
$ ros2 topic list -v
Published topics:
 * /ap/battery/battery0 [sensor_msgs/msg/BatteryState] 1 publisher
 * /ap/clock [rosgraph_msgs/msg/Clock] 1 publisher
 * /ap/geopose/filtered [geographic_msgs/msg/GeoPoseStamped] 1 publisher
 * /ap/gps_global_origin/filtered [geographic_msgs/msg/GeoPointStamped] 1 publisher
 * /ap/imu/experimental/data [sensor_msgs/msg/Imu] 1 publisher
 * /ap/navsat/navsat0 [sensor_msgs/msg/NavSatFix] 1 publisher
 * /ap/pose/filtered [geometry_msgs/msg/PoseStamped] 1 publisher
 * /ap/tf_static [tf2_msgs/msg/TFMessage] 1 publisher
 * /ap/time [builtin_interfaces/msg/Time] 1 publisher
 * /ap/twist/filtered [geometry_msgs/msg/TwistStamped] 1 publisher
 * /parameter_events [rcl_interfaces/msg/ParameterEvent] 1 publisher
 * /rosout [rcl_interfaces/msg/Log] 1 publisher

Subscribed topics:
 * /ap/cmd_gps_pose [ardupilot_msgs/msg/GlobalPosition] 1 subscriber
 * /ap/cmd_vel [geometry_msgs/msg/TwistStamped] 1 subscriber
 * /ap/joy [sensor_msgs/msg/Joy] 1 subscriber
 * /ap/tf [tf2_msgs/msg/TFMessage] 1 subscriber
```

```bash
$ ros2 topic hz /ap/time
average rate: 50.115
        min: 0.012s max: 0.024s std dev: 0.00328s window: 52
```

```bash
$ ros2 topic echo /ap/time
sec: 1678668735
nanosec: 729410000
```

```bash
$ ros2 service list
/ap/arm_motors
/ap/mode_switch
---
```

The static transforms for enabled sensors are also published, and can be received like so:

```bash
ros2 topic echo /ap/tf_static --qos-depth 1 --qos-history keep_last --qos-reliability reliable --qos-durability transient_local --once
```
This conversion  of data allows to connect both drone and ROS2 seamlesly.
In order to consume the transforms, it's highly recommended to [create and run a transform broadcaster in ROS 2](https://docs.ros.org/en/humble/Concepts/About-Tf2.html#tutorials).


## Info
1. TF frames of the simulation.
![image](https://github.com/user-attachments/assets/d0768435-7fd6-4ac3-8513-62273b8e2ce7)

2. rqt_graph
![anas](https://github.com/user-attachments/assets/4501180e-c8b8-4015-9d1d-d27d49410e6e)


