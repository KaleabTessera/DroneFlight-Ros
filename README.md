# DroneFlight-Ros
## How to run
### Startup simulation
1. source /opt/ros/kinetic/setup.bash (Depending on where ROS is installed)
2. source devel/setup.bash
3. roslaunch cvg_sim_gazebo ardrone_testworld.launch

### Run PID controller
1. source /opt/ros/kinetic/setup.bash (Depending on where ROS is installed)
2. source devel/setup.bash
3. roscd drone_controller
4. rosrun drone_controller dronecontroller.py
