# ars_path_follower

@settings {
  font-size: 70;
}

Results: images

![alt text](https://github.com/Moado/Robotics-ROS/blob/main/Homework5/images/pf_1.PNG?raw=true)

![alt text](https://github.com/Moado/Robotics-ROS/blob/main/Homework5/images/pf_2.PNG?raw=true)


Videos and experimentations: 


https://user-images.githubusercontent.com/23427415/144330137-ba81f6a9-e246-4a92-bdbf-b808b5b44863.mp4



https://user-images.githubusercontent.com/23427415/144330148-0ecdbffc-c000-437c-8b46-9a1fb4a13e53.mp4



https://user-images.githubusercontent.com/23427415/144330151-6ae8e672-8ca6-4d9c-bd0e-1b70089666bc.mp4



https://user-images.githubusercontent.com/23427415/144330157-627e8a00-3b77-4449-9229-c3ed46a3b963.mp4



https://user-images.githubusercontent.com/23427415/144330163-4159d5f2-2859-408f-925f-822fc123e89f.mp4



cd $ARS_CATKIN_WORKSPACE </br >
source $ARS_CATKIN_WORKSPACE/devel/setup.bash </br >
rospack profile </br >
roscore </br >

roslaunch ars_launchers robot_simulator.launch </br >

rosrun rviz rviz -d $(rospack find ars_config)/config/rviz_config/rviz_conf_sim.rviz </br >

roslaunch ars_launchers robot_trajectory.launch </br >

roslaunch ars_launchers environment_simulator.launch environment_description_yaml_file:="$(rospack find ars_config)/config/environment/obstacles_env_01.yaml" </br >

#rostopic pub -1 /simulator/sim_environment/flag_dynamic_obstacles std_msgs/Bool "data: true" </br >

roslaunch ars_launchers obstacles_detector_simulator.launch </br >

roslaunch ars_launchers robot_simulator_sensors_robot.launch </br >

roslaunch ars_launchers robot_obstacle_avoidance_react.launch </br >

roslaunch ars_launchers robot_msf_state_estimator.launch </br >

roslaunch ars_launchers robot_trajectory_estim.launch </br >

roslaunch ars_launchers robot_motion_controller_pid.launch robot_cmd_ctr_stamped:=/robot_cmd_ctr_stamped robot_cmd_ctr:=/robot_cmd_ctr flag_use_state_estim:=True </br >

roslaunch ars_launchers robot_path_follower.launch flag_use_state_estim:=True </br >

bash $ARS_PROJECT/scripts/publish_path_01.sh
