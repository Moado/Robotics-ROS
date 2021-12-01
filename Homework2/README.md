# ars_obstacle_avoidance_react

@settings {
  font-size: 70;
}


https://user-images.githubusercontent.com/23427415/136907775-add1500c-5c30-4d49-bc73-7920c73f9cfc.mp4




https://user-images.githubusercontent.com/23427415/144329862-913053d3-7386-4d39-bf67-9efff0f8e5b7.mp4



source $ARS_CATKIN_WORKSPACE/devel/setup.bash <br />
rospack profile <br />
roscore 


source $ARS_CATKIN_WORKSPACE/devel/setup.bash <br />
rospack profile <br />
roslaunch ars_launchers robot_simulator.launch


source $ARS_CATKIN_WORKSPACE/devel/setup.bash <br />
rospack profile <br />
roslaunch ars_robot_models robot_urdf.launch


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
roslaunch ars_launchers robot_remote_controller.launch


source $ARS_CATKIN_WORKSPACE/devel/setup.bash <br />
rospack profile <br />
roslaunch ars_launchers robot_obstacle_avoidance_react.launch



#move in circles <br />

source $ARS_CATKIN_WORKSPACE/devel/setup.bash <br />
rospack profile <br />

![alt text](https://github.com/Moado/Robotics-ROS/blob/main/Homework2/images/cercle.PNG?raw=true)



#to stop the robot <br />
source $ARS_CATKIN_WORKSPACE/devel/setup.bash <br />
rospack profile <br />

![alt text](https://github.com/Moado/Robotics-ROS/blob/main/Homework2/images/stop.PNG?raw=true)
