# ars_path_planner

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