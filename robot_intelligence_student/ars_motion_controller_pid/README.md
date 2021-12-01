# ars_motion_controller_pid

# ars_msf_state_estimator


cd $ARS_CATKIN_WORKSPACE

catkin clean
catkin build


cd $ARS_CATKIN_WORKSPACE
source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roscore 

cd $ARS_CATKIN_WORKSPACE
source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roslaunch ars_launchers robot_simulator.launch


cd $ARS_CATKIN_WORKSPACE
source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
rosrun rviz rviz -d $(rospack find ars_config)/config/rviz_config/rviz_conf_sim.rviz


cd $ARS_CATKIN_WORKSPACE
source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roslaunch ars_launchers environment_simulator.launch environment_description_yaml_file:="$(rospack find ars_config)/config/environment/obstacles_env_02.yaml"

cd $ARS_CATKIN_WORKSPACE
source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roslaunch ars_launchers obstacles_detector_simulator.launch

cd $ARS_CATKIN_WORKSPACE
source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roslaunch ars_launchers robot_simulator_sensors_robot.launch


cd $ARS_CATKIN_WORKSPACE
source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roslaunch ars_launchers robot_obstacle_avoidance_react.launch


cd $ARS_CATKIN_WORKSPACE
source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roslaunch ars_launchers robot_msf_state_estimator.launch

cd $ARS_CATKIN_WORKSPACE
source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roslaunch ars_launchers robot_motion_controller_pid.launch robot_cmd_ctr_stamped:=/robot_cmd_ctr_stamped robot_cmd_ctr:=/robot_cmd_ctr flag_use_state_estim:=True
