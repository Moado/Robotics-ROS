# ars_remote_controller_improved

This version of the code is the one I will use in all the rest of the project. It is an improved version of the remote controller, which includes in addition to the graphical interface, 3 other scripts:

twistStampedtotwist.py which allows me to visualize the linear and angular change and the steering of the robot, directly on the terminal. 
![alt text](https://github.com/Moado/Robotics-ROS/blob/main/Homework2/images/ars.gif?raw=true)

go_in_circle.sh which is a bash script that allows the robot to turn infinitely in a circle (we can however add the "r" tag for recursivity).
![alt text](https://github.com/Moado/Robotics-ROS/blob/main/Homework2/images/cercle.PNG?raw=true)
stop_robot.sh which allows the robot to stop instantly when it goes out of its trajectory. 
![alt text](https://github.com/Moado/Robotics-ROS/blob/main/Homework2/images/stop.PNG?raw=true)


@settings {
  font-size: 70;
}


The order of the commands is as follows:


cd $ARS_CATKIN_WORKSPACE <br />
catkin clean <br />
catkin build <br />

source $ARS_CATKIN_WORKSPACE/devel/setup.bash <br />
rospack profile <br />
roscore 


source $ARS_CATKIN_WORKSPACE/devel/setup.bash <br />
rospack profile <br />
roslaunch ars_sim_robot ars_sim_robot.launch 


source $ARS_CATKIN_WORKSPACE/devel/setup.bash <br />
rospack profile <br />
roslaunch ars_robot_models robot_urdf.launch               


source $ARS_CATKIN_WORKSPACE/devel/setup.bash <br />
rospack profile <br />
rosrun rviz rviz -d $(rospack find ars_config)/config/rviz_config/rviz_conf_sim.rviz


source $ARS_CATKIN_WORKSPACE/devel/setup.bash <br />
rospack profile <br />
roslaunch ars_launchers robot_trajectory.launch 


source $ARS_CATKIN_WORKSPACE/devel/setup.bash <br />
rospack profile <br />
rosrun ars_remote_controller ars_control.py


rosbag record -a <br />


source $ARS_CATKIN_WORKSPACE/devel/setup.bash <br />
rospack profile


source $ARS_CATKIN_WORKSPACE/devel/setup.bash <br />
rospack profile <br />
rqt_plot /robot_pose/pose/position/x:y:z 


source $ARS_CATKIN_WORKSPACE/devel/setup.bash <br />
rospack profile <br />
rosrun ars_remote_controller graphical_interface.py


rostopic echo /robot_cmd_ctr_stamped or /robot_cmd_stamped (depending on if we're connecting the reactive avoidance package or not)

Fresh start of the terminals and execution of : "rosbag play name.bag" to simulate the recorded scenario.


Credit: 

[1] https://github.com/Tanguyvans/bebop_code "Original source code"

[2] https://sceweb.sce.uhcl.edu/harman/CENG_all/TurtleBotGuide2_19_2016a.pdf "Turtle Bot Guide"

[3] https://www.youtube.com/watch?v=eJ4QPrYqMlw "Youtube video"

[4] https://femexrobotica.org/eir2015/wp-content/uploads/2015/01/Navegaci%C3%B3nDeRobotsM%C3%B3viles.pdf "Lecture notes"
