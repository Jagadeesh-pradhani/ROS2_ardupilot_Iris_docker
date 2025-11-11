# NON-GPS

# T1
ros2 launch ardupilot_gz_bringup iris_maze.launch.py lidar_dim:=2

# T2
ros2 launch ardupilot_cartographer cartographer.launch.py rviz:=false

# T3
ros2 launch ardupilot_cartographer navigation.launch.py

# T4
mavproxy.py --console --map --aircraft test --master=:14550

# T5
ros2 launch octomap_server octomap_mapping.launch.xml

# Wait untill GPS fix and then plan in nav2


# Octomap subscribe to /d_scan/points




####################
https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos
https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos