# ars_motion_controller_pid


https://user-images.githubusercontent.com/23427415/139527432-1f72703b-a01c-4722-b932-7d1566204603.mp4


***commands:*** <br />

cd $ARS_CATKIN_WORKSPACE <br />

catkin clean <br />
catkin build


source $ARS_CATKIN_WORKSPACE/devel/setup.bash <br />
rospack profile <br />
roscore 

source $ARS_CATKIN_WORKSPACE/devel/setup.bash <br />
rospack profile <br />
roslaunch ars_launchers robot_simulator.launch


source $ARS_CATKIN_WORKSPACE/devel/setup.bash <br />
rospack profile <br />
rosrun rviz rviz -d $(rospack find ars_config)/config/rviz_config/rviz_conf_sim.rviz


source $ARS_CATKIN_WORKSPACE/devel/setup.bash <br />
rospack profile <br />
roslaunch ars_launchers environment_simulator.launch environment_description_yaml_file:="$(rospack find ars_config)/config/environment/obstacles_env_02.yaml"


source $ARS_CATKIN_WORKSPACE/devel/setup.bash <br />
rospack profile <br />
roslaunch ars_launchers obstacles_detector_simulator.launch


source $ARS_CATKIN_WORKSPACE/devel/setup.bash <br />
rospack profile <br />
roslaunch ars_launchers robot_simulator_sensors_robot.launch



source $ARS_CATKIN_WORKSPACE/devel/setup.bash <br />
rospack profile <br />
roslaunch ars_launchers robot_obstacle_avoidance_react.launch



source $ARS_CATKIN_WORKSPACE/devel/setup.bash <br />
rospack profile <br />
roslaunch ars_launchers robot_msf_state_estimator.launch


source $ARS_CATKIN_WORKSPACE/devel/setup.bash <br />
rospack profile <br />
roslaunch ars_launchers robot_motion_controller_pid.launch robot_cmd_ctr_stamped:=/robot_cmd_ctr_stamped robot_cmd_ctr:=/robot_cmd_ctr flag_use_state_estim:=True
