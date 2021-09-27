# ars_remote_controller

Based on existing scripts, I created and adapted an interface to control the drone. This work will be improved as the project of creating an autonomous drone progresses. 

I used the first version of the code you gave us last year because in the recent version I encountered many difficulties. So I will work with the first version of the code and add the other components progressively. 

The order of the commands is as follows:

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

Fresh start of the terminals and execution of : "rosbag play name.bag" to simulate the recorded scenario.


Credit: 

1- https://github.com/Tanguyvans/bebop_code 

2- https://sceweb.sce.uhcl.edu/harman/CENG_all/TurtleBotGuide2_19_2016a.pdf

3- https://www.youtube.com/watch?v=eJ4QPrYqMlw

4- https://femexrobotica.org/eir2015/wp-content/uploads/2015/01/Navegaci%C3%B3nDeRobotsM%C3%B3viles.pdf 
