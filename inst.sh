Follow the instruction below as stated in '#' comments.


###############################################################################
#Dependecies

sudo apt install default-jre

git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
cd Micro-XRCE-DDS-Gen
./gradlew assemble

-------------------------------------------
#Add the following line to .bashrc file and source
gedit ~/.bashrc
export PATH=$PATH:~/Micro-XRCE-DDS-Gen/scripts
source ~/.bashrc
-------------------------------------------

sudo apt-get update
sudo apt-get install git
sudo apt-get install gitk git-gui
sudo apt-get install gcc-arm-none-eabi

###############################################################################
#Ardupilot 

#Extract the ardupilot folder and paste it in home location.
cd ~/ardupilot


./waf distclean
./waf distclean
./waf configure --board MatekF405-Wing
./waf plane


-------------------------------------------
#If fail, please try:

./waf clean
./waf distclean
./waf configure --board MatekF405-Wing
-------------------------------------------


###############################################################################
#ROS2

#Create the work space
mkdir -p ~/ros2_ws/src

cd ~/ros2_ws
vcs import --recursive --input  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src
sudo apt update
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src

#build workspace

cd ~/ros2_ws
colcon build --packages-up-to ardupilot_dds_tests

------------------------------------------------------------------------------
#If the build fails,  please re-run the build in verbose mode like so:

colcon build --packages-up-to ardupilot_dds_tests --event-handlers=console_cohesion+
------------------------------------------------------------------------------

#test
cd ~/ros2_ws
source ./install/setup.bash
colcon test --packages-select ardupilot_dds_tests
colcon test-result --all --verbose

#SITL
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y #
./waf clean
./waf configure --board sitl
./waf copter -v

cd ~/ardupilot/Tools/autotest

sudo pip3 install MAVProxy
mavproxy.py --version

-------------------------------------------
#Add the following line to .bashrc file and source
gedit ~/.bashrc
export PATH=$PATH:/path/to/mavproxy
source ~/.bashrc
-------------------------------------------


------------------------------------------------------------------------
#####TESTING
#Run the following commands to test SITL with different options, after running close all cleanly using ctrl+c
./sim_vehicle.py -v ArduCopter -w

./sim_vehicle.py -v ArduCopter --console --map

./sim_vehicle.py -v ArduCopter -L KSFO --console --map

./sim_vehicle.py -v ArduPlane -f quadplane --console --map --osd

./sim_vehicle.py -v ArduCopter -f quadcopter --console --map --osd
------------------------------------------------------------------------


###############################################################################
#ROS2 with SITL
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/
colcon build --packages-up-to ardupilot_sitl
source ~/ros2_ws/install/setup.bash

###TESTING
ros2 launch ardupilot_sitl sitl_dds_udp.launch.py transport:=udp4 refs:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/dds_xrce_profile.xml synthetic_clock:=True wipe:=False model:=quad speedup:=1 slave:=0 instance:=0 defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_udp.parm sim_address:=127.0.0.1 master:=tcp:127.0.0.1:5760 sitl:=127.0.0.1:5501

#This above command will load the sitl with different connecrtion ports, close with ctrl+c

###############################################################################
#ROS2 with SITL in GAZEBO
cd ~/ros2_ws

-------------------------------------------
#Add the following line to .bashrc file and source  
gedit ~/.bashrc
export GZ_VERSION=harmonic
source ~/.bashrc
-------------------------------------------

cd ~/ros2_ws
vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src
source /opt/ros/humble/setup.bash
sudo apt update
rosdep update
rosdep install -y --from-paths src --ignore-src -r

cd ~/ros2_ws
colcon build --packages-up-to ardupilot_gz_bringup

cd ~/ros2_ws
source install/setup.bash
colcon test --packages-select ardupilot_sitl ardupilot_dds_tests ardupilot_gazebo ardupilot_gz_applications ardupilot_gz_description ardupilot_gz_gazebo ardupilot_gz_bringup
colcon test-result --all --verbose





###############################################################################
##TESING  : Final simulation 

#Replace the 'iris_bridge.yaml' file inside 'config' folder at ~/ros2_ws/src/ardupilot_gz/ardupilot_gz_bringup/config/
#Replace the 'iris_with_gimbal' folder inside 'models' folder at ~/ros2_ws/src/ardupilot_gazebo/models/
#Replace the 'iris_with_lidar' folder inside 'models' folder at ~/ros2_ws/src/ardupilot_gz/ardupilot_gz_description/models/


#Terminal-1
cd ~/ros2_ws
source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py

#Terminal-2
mavproxy.py --console --map --aircraft test --master=:14550




