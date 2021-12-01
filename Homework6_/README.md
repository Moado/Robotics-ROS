# ars_path_planner

![alt text](https://github.com/Moado/Robotics-ROS/blob/main/Homework6_/images/pp_1.PNG?raw=true)

![alt text](https://github.com/Moado/Robotics-ROS/blob/main/Homework6_/images/pp_2.PNG?raw=true)

![alt text](https://github.com/Moado/Robotics-ROS/blob/main/Homework6_/images/pp_3.PNG?raw=true)


![path_planner1](https://user-images.githubusercontent.com/23427415/144330522-e6e2e600-c621-4edf-8c20-4e830e3edb0d.PNG)
![path_planner2](https://user-images.githubusercontent.com/23427415/144330531-4974395e-b36f-44b4-8c18-758f793b79aa.PNG)
![path_planner3](https://user-images.githubusercontent.com/23427415/144330532-17853809-2828-4c9e-9d49-b528c359dbc4.PNG)


cd $ARS_CATKIN_WORKSPACE
source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roscore

roslaunch ars_launchers robot_simulator.launch

rosrun rviz rviz -d $(rospack find ars_config)/config/rviz_config/rviz_conf_sim.rviz

roslaunch ars_launchers robot_trajectory.launch

roslaunch ars_launchers environment_simulator.launch environment_description_yaml_file:="$(rospack find ars_config)/config/environment/obstacles_env_01.yaml"

#rostopic pub -1 /simulator/sim_environment/flag_dynamic_obstacles std_msgs/Bool "data: true"

roslaunch ars_launchers obstacles_detector_simulator.launch

roslaunch ars_launchers robot_simulator_sensors_robot.launch

roslaunch ars_launchers mapper_simulator.launch

roslaunch ars_launchers robot_obstacle_avoidance_react.launch

roslaunch ars_launchers robot_msf_state_estimator.launch

roslaunch ars_launchers robot_trajectory_estim.launch

roslaunch ars_launchers robot_motion_controller_pid.launch robot_cmd_ctr_stamped:=/robot_cmd_ctr_stamped robot_cmd_ctr:=/robot_cmd_ctr flag_use_state_estim:=True

roslaunch ars_launchers robot_path_follower.launch flag_use_state_estim:=True

roslaunch ars_launchers robot_path_planner.launch flag_use_state_estim:=True

#example (7 5) ; (1 7) ; (0 0) ; (6.5 5.5)
