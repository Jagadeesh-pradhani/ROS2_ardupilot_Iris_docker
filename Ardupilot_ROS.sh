#source
source ~/.bashrc

#Ardupilot
------------------------------------------------------------------------
#####TESTING
#Run the following commands to test SITL with different options, after running close all cleanly using ctrl+c
cd ~/ardupilot
./sim_vehicle.py -v ArduCopter -w

./sim_vehicle.py -v ArduCopter --console --map

./sim_vehicle.py -v ArduCopter -L KSFO --console --map

./sim_vehicle.py -v ArduPlane -f quadplane --console --map --osd

./sim_vehicle.py -v ArduCopter -f quadcopter --console --map --osd
------------------------------------------------------------------------

#ROS with SITL TESTING
cd ~/ros2_ws
source install/setup.bash
ros2 launch ardupilot_sitl sitl_dds_udp.launch.py transport:=udp4 refs:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/dds_xrce_profile.xml synthetic_clock:=True wipe:=False model:=quad speedup:=1 slave:=0 instance:=0 defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_udp.parm sim_address:=127.0.0.1 master:=tcp:127.0.0.1:5760 sitl:=127.0.0.1:5501


##TESING  : Final simulation 

#Terminal-1 (docker run -it --user ros --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY ros_humble)
cd ~/ros2_ws
source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py

#Terminal-2 (docker exec -it naughty_shannon /bin/bash) change name according to container
mavproxy.py --console --map --aircraft test --master=:14550