# ars_obstacle_avoidance_react


source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roscore 


source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roslaunch ars_launchers robot_simulator.launch


source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roslaunch ars_robot_models robot_urdf.launch


source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
rosrun rviz rviz -d $(rospack find ars_config)/config/rviz_config/rviz_conf_sim.rviz


source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roslaunch ars_launchers environment_simulator.launch environment_description_yaml_file:="$(rospack find ars_config)/config/environment/obstacles_env_02.yaml"



source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roslaunch ars_launchers obstacles_detector_simulator.launch


source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roslaunch ars_launchers robot_remote_controller.launch


source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roslaunch ars_launchers robot_obstacle_avoidance_react.launch



#move in circles

source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile

![alt text](https://github.com/Moado/Robotics-ROS/blob/main/Homework2/images/cercle.PNG?raw=true)



#to stop the robot
source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile

![alt text](https://github.com/Moado/Robotics-ROS/blob/main/Homework2/images/stop.PNG?raw=true)
