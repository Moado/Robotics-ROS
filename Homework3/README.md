# ars_msf_state_estimator

@settings {
  font-size: 70;
}


https://user-images.githubusercontent.com/23427415/138777439-ace22039-9101-4452-be70-d44960a0d967.mp4


[Before Tuning]
![alt text](https://github.com/Moado/Robotics-ROS/blob/main/Homework3/images/state-estimator-before-tuning.PNG?raw=true)


[After Tuning]
![alt text](https://github.com/Moado/Robotics-ROS/blob/main/Homework3/images/state-estimator-after-tuning.PNG?raw=true)


Commands:

cd $ARS_CATKIN_WORKSPACE

catkin clean
catkin build



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
roslaunch ars_launchers robot_trajectory.launch

source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roslaunch ars_launchers environment_simulator.launch environment_description_yaml_file:="$(rospack find ars_config)/config/environment/obstacles_env_00.yaml"

source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roslaunch ars_launchers obstacles_detector_simulator.launch

source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roslaunch ars_launchers robot_simulator_sensors_robot.launch

source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roslaunch ars_launchers robot_remote_controller.launch

source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roslaunch ars_launchers robot_obstacle_avoidance_react.launch


source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roslaunch ars_launchers robot_trajectory_estim.launch

source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roslaunch ars_launchers robot_msf_state_estimator.launch



