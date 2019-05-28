# DroneFlight-Ros
## How to run
### Startup simulation
1. ```source /opt/ros/kinetic/setup.bash``` (Depending on where ROS is installed)
2. ```source devel/setup.bash```
3. ```roslaunch cvg_sim_gazebo ardrone_testworld.launch```

### Run PID controller
1. ```source /opt/ros/kinetic/setup.bash``` (Depending on where ROS is installed)
2. ```source devel/setup.bash```
3. ```roscd drone_controller```
4. Run Controller
- ```rosrun drone_controller dronecontroller.py --dest x,y,z``` Default x,y,z - 3,2,5 
`OR`
- ```python dronecontroller.py --dest x,y,z``` Default x,y,z - 3,2,5 

Example ```rosrun drone_controller dronecontroller.py --dest 3,2,5```
