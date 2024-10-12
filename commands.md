## Dependecies
```

sudo apt install default-jre

git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
cd Micro-XRCE-DDS-Gen
./gradlew assemble
```

```
echo "export PATH=$PATH:~/Micro-XRCE-DDS-Gen/scripts" >> ~/.bashrc
```
```
sudo apt-get update
sudo apt-get install git
sudo apt-get install gitk git-gui
sudo apt-get install gcc-arm-none-eabi
```

## Ardupilot

```
cd
git clone https://github.com/ArduPilot/ardupilot.git
git submodule update --init --recursive
```

```
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y #
./waf clean
./waf configure --board sitl
./waf copter -v
```
```
cd ~/ardupilot/Tools/autotest

sudo pip3 install MAVProxy
```

### Test
```
./sim_vehicle.py -v ArduCopter -f quadcopter --console --map --osd
```

## ros2_ws

```
mkdir -p ~/ros2_ws/src

cd ~/ros2_ws
vcs import --recursive --input  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src
sudo apt update
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src
```

```
cd ~/ros2_ws
colcon build --packages-up-to ardupilot_dds_tests
```

```
cd ~/ros2_ws
source ./install/setup.bash
colcon test --packages-select ardupilot_dds_tests
colcon test-result --all --verbose
```
```
cd ~/ros2_ws/
colcon build --packages-up-to ardupilot_sitl
source ~/ros2_ws/install/setup.bash
```

```
ros2 launch ardupilot_sitl sitl_dds_udp.launch.py transport:=udp4 refs:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/dds_xrce_profile.xml synthetic_clock:=True wipe:=False model:=quad speedup:=1 slave:=0 instance:=0 defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_udp.parm sim_address:=127.0.0.1 master:=tcp:127.0.0.1:5760 sitl:=127.0.0.1:5501
```
```
echo "export GZ_VERSION=garden" >> ~/.bashrc
```
```
cd ~/ros2_ws
vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src
source /opt/ros/humble/setup.bash
sudo apt update
rosdep update
rosdep install -y --from-paths src --ignore-src -r
colcon build --packages-up-to ardupilot_gz_bringup
```

```
cd ~/ros2_ws
source install/setup.bash
colcon test --packages-select ardupilot_sitl ardupilot_dds_tests ardupilot_gazebo ardupilot_gz_applications ardupilot_gz_description ardupilot_gz_gazebo ardupilot_gz_bringup
colcon test-result --all --verbose
```

## Test
```
cd ~/ros2_ws
source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

```
mavproxy.py --console --map --aircraft test --master=:14550
```




