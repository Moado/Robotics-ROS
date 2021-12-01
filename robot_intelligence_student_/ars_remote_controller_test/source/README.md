# ars_remote_controller_test

cd $ARS_CATKIN_WORKSPACE
source $ARS_CATKIN_WORKSPACE/devel/setup.bash
rospack profile
roscore

roslaunch ars_sim_robot ars_sim_robot.launch


roslaunch ars_robot_models robot_urdf.launch               


rosrun rviz rviz -d $(rospack find ars_config)/config/rviz_config/rviz_conf_sim.rviz


roslaunch ars_launchers robot_trajectory.launch


roslaunch ars_launchers robot_remote_controller_test.launch


rosbag record -a


rqt_plot /robot_pose/pose/position/:x:y:z 


rosrun ars_remote_controller_test graphical_interface.py
rosrun ars_remote_controller_test twistStampedtotwist.py

go to path 
cd ~/workspace/catkin_ws/src/ars_project/packages/robot_intelligence_student/ars_remote_controller_test/source 
and run: "bash go_in_cirle.sh"


rostopic echo /robot_cmd_stamped


refresh and run: rosbag play name.bag


Credit: 
https://github.com/Tanguyvans/bebop_code 
https://sceweb.sce.uhcl.edu/harman/CENG_all/TurtleBotGuide2_19_2016a.pdf
https://www.youtube.com/watch?v=eJ4QPrYqMlw
https://femexrobotica.org/eir2015/wp-content/uploads/2015/01/Navegaci%C3%B3nDeRobotsM%C3%B3viles.pdf 