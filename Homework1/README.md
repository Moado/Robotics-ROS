# ars_remote_controller

cd $ARS_CATKIN_WORKSPACE
catkin_make
source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roscore

source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roslaunch ars_sim_robot ars_sim_robot.launch

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
rosrun ars_remote_controller /ars_control.py

rosbag record -a

source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.5, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
rqt_plot /robot_pose/pose/position/x:y:z 

source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
rosrun ars_remote_controller /graphical_interface.py

rostopic echo /cmd_vel

refresh and run: rosbag play name.bag

Credit: 

1- https://github.com/Tanguyvans/bebop_code 
2- https://sceweb.sce.uhcl.edu/harman/CENG_all/TurtleBotGuide2_19_2016a.pdf
3- https://www.youtube.com/watch?v=eJ4QPrYqMlw
4- https://femexrobotica.org/eir2015/wp-content/uploads/2015/01/Navegaci%C3%B3nDeRobotsM%C3%B3viles.pdf 
