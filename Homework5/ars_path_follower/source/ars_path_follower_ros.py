#!/usr/bin/env python

import numpy as np
from numpy import *

import os




# ROS

import rospy

import rospkg

import std_msgs.msg
from std_msgs.msg import Header

import geometry_msgs.msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

import nav_msgs.msg
from nav_msgs.msg import Path

import tf_conversions

import tf2_ros




#
from ars_path_follower import *


#
import ars_lib_helpers





class ArsPathFollowerRos:

  #######


  # Robot frame
  robot_frame = 'robot_base_link'

  # World frame
  world_frame = 'world'



  # Ctr command loop freq 
  # time step
  ctr_cmd_loop_freq = 10.0
  # Timer
  ctr_cmd_loop_timer = None



  # Robot traj subscriber
  robot_traj_sub = None

  # Robot pose subscriber
  robot_pose_sub = None
  # Robot velocity subscriber
  robot_vel_world_sub = None


  # Robot pose ref pub
  robot_pose_ref_pub = None
  # Robot velocity ref pub
  robot_vel_world_ref_pub = None
  #
  robot_vel_cmd_ref_pub = None


  # Motion controller
  path_follower = ArsPathFollower()
  


  #########

  def __init__(self):

    # Thread.__init__(self)
    # self.stopped = event
    # Motion controller
    self.path_follower = ArsPathFollower()


    # end
    return


  def init(self, node_name='ars_path_follower_node'):
    #

    # Init ROS
    rospy.init_node(node_name, anonymous=True)

    
    # Package path
    pkg_path = rospkg.RosPack().get_path('ars_path_follower')
    

    #### READING PARAMETERS ###
    
    # TODO

    ###

    
    # End
    return


  def open(self):

    # Subscribers

    #
    self.robot_traj_sub = rospy.Subscriber('robot_trajectory_ref', Path, self.robotTrajectoryCallback)

    # 
    self.robot_pose_sub = rospy.Subscriber('robot_pose', PoseStamped, self.robotPoseCallback)
    #
    self.robot_vel_world_sub = rospy.Subscriber('robot_velocity_world', TwistStamped, self.robotVelWorldCallback)
    


    # Publishers

    # 
    self.robot_pose_ref_pub = rospy.Publisher('robot_pose_ctr_ref', PoseStamped, queue_size=1)
    #
    self.robot_vel_world_ref_pub = rospy.Publisher('robot_velocity_world_ctr_ref', TwistStamped, queue_size=1)
    #
    self.robot_vel_cmd_ref_pub = rospy.Publisher('robot_ctr_cmd_ref', TwistStamped, queue_size=1)




    # Timers
    #
    self.ctr_cmd_loop_timer = rospy.Timer(rospy.Duration(1.0/self.ctr_cmd_loop_freq), self.ctrCommandLoopTimerCallback)


    # End
    return


  def run(self):

    rospy.spin()

    return


  def robotTrajectoryCallback(self, robot_trajectory_msg):

    # 
    robot_trajectory = []

    #
    for robot_trajectory_pose_i_msg in robot_trajectory_msg.poses:

      robot_trajectory_pose_i = ars_lib_helpers.PoseSimp()

      robot_trajectory_pose_i.position[0] = robot_trajectory_pose_i_msg.pose.position.x
      robot_trajectory_pose_i.position[1] = robot_trajectory_pose_i_msg.pose.position.y
      robot_trajectory_pose_i.position[2] = robot_trajectory_pose_i_msg.pose.position.z

      quat_i = ars_lib_helpers.Quaternion.zerosQuat()
      quat_i[0] = robot_trajectory_pose_i_msg.pose.orientation.w
      quat_i[1] = robot_trajectory_pose_i_msg.pose.orientation.x
      quat_i[2] = robot_trajectory_pose_i_msg.pose.orientation.y
      quat_i[3] = robot_trajectory_pose_i_msg.pose.orientation.z

      quat_i = ars_lib_helpers.Quaternion.normalize(quat_i)

      robot_trajectory_pose_i.attitude_quat_simp = ars_lib_helpers.Quaternion.getSimplifiedQuatRobotAtti(quat_i)

      robot_trajectory.append(robot_trajectory_pose_i)

    #
    self.path_follower.setRobotTrajectory(robot_trajectory)


    # End
    return


  def robotPoseCallback(self, robot_pose_msg):

    # Position
    robot_posi = np.zeros((3,), dtype=float)
    robot_posi[0] = robot_pose_msg.pose.position.x
    robot_posi[1] = robot_pose_msg.pose.position.y
    robot_posi[2] = robot_pose_msg.pose.position.z

    # Attitude quat simp
    robot_atti_quat = ars_lib_helpers.Quaternion.zerosQuat()
    robot_atti_quat[0] = robot_pose_msg.pose.orientation.w
    robot_atti_quat[1] = robot_pose_msg.pose.orientation.x
    robot_atti_quat[2] = robot_pose_msg.pose.orientation.y
    robot_atti_quat[3] = robot_pose_msg.pose.orientation.z

    robot_atti_quat_simp = ars_lib_helpers.Quaternion.getSimplifiedQuatRobotAtti(robot_atti_quat)

    #
    self.path_follower.setRobotPose(robot_posi, robot_atti_quat_simp)

    #
    return


  def robotVelWorldCallback(self, robot_vel_msg):

    # Linear
    lin_vel_world = np.zeros((3,), dtype=float)
    lin_vel_world[0] = robot_vel_msg.twist.linear.x
    lin_vel_world[1] = robot_vel_msg.twist.linear.y
    lin_vel_world[2] = robot_vel_msg.twist.linear.z

    # Angular
    ang_vel_world = np.zeros((1,), dtype=float)
    ang_vel_world[0] = robot_vel_msg.twist.angular.z

    #
    self.path_follower.setRobotVelWorld(lin_vel_world, ang_vel_world)

    #
    return


  def robotPoseRefPublish(self, time_stamp_current):

    robot_posi_ref = self.path_follower.robot_posi_ref
    robot_atti_quat_simp_ref = self.path_follower.robot_atti_quat_simp_ref

    robot_pose_ref_msg = PoseStamped()

    robot_pose_ref_msg.header.stamp = time_stamp_current
    robot_pose_ref_msg.header.frame_id = self.world_frame

    robot_pose_ref_msg.pose.position.x = robot_posi_ref[0]
    robot_pose_ref_msg.pose.position.y = robot_posi_ref[1]
    robot_pose_ref_msg.pose.position.z = robot_posi_ref[2]

    robot_pose_ref_msg.pose.orientation.w = robot_atti_quat_simp_ref[0]
    robot_pose_ref_msg.pose.orientation.x = 0.0
    robot_pose_ref_msg.pose.orientation.y = 0.0
    robot_pose_ref_msg.pose.orientation.z = robot_atti_quat_simp_ref[1]


    self.robot_pose_ref_pub.publish(robot_pose_ref_msg)


    return


  def robotVelocityRefPublish(self, time_stamp_current):

    robot_velo_lin_world_ref = self.path_follower.robot_velo_lin_world_ref
    robot_velo_ang_world_ref = self.path_follower.robot_velo_ang_world_ref

    robot_velo_ref_msg = TwistStamped()

    robot_velo_ref_msg.header.stamp = time_stamp_current
    robot_velo_ref_msg.header.frame_id = self.world_frame

    robot_velo_ref_msg.twist.linear.x = robot_velo_lin_world_ref[0]
    robot_velo_ref_msg.twist.linear.y = robot_velo_lin_world_ref[1]
    robot_velo_ref_msg.twist.linear.z = robot_velo_lin_world_ref[2]

    robot_velo_ref_msg.twist.angular.x = 0.0
    robot_velo_ref_msg.twist.angular.y = 0.0
    robot_velo_ref_msg.twist.angular.z = robot_velo_ang_world_ref[0]


    self.robot_vel_world_ref_pub.publish(robot_velo_ref_msg)


    return


  def robotVelCmdRefPublish(self, time_stamp_current):

    robot_velo_lin_cmd_ref = self.path_follower.robot_velo_lin_cmd_ref
    robot_velo_ang_cmd_ref = self.path_follower.robot_velo_ang_cmd_ref

    robot_velo_cmd_ref_msg = TwistStamped()

    robot_velo_cmd_ref_msg.header.stamp = time_stamp_current
    robot_velo_cmd_ref_msg.header.frame_id = self.world_frame

    robot_velo_cmd_ref_msg.twist.linear.x = robot_velo_lin_cmd_ref[0]
    robot_velo_cmd_ref_msg.twist.linear.y = robot_velo_lin_cmd_ref[1]
    robot_velo_cmd_ref_msg.twist.linear.z = robot_velo_lin_cmd_ref[2]

    robot_velo_cmd_ref_msg.twist.angular.x = 0.0
    robot_velo_cmd_ref_msg.twist.angular.y = 0.0
    robot_velo_cmd_ref_msg.twist.angular.z = robot_velo_ang_cmd_ref[0]


    self.robot_vel_cmd_ref_pub.publish(robot_velo_cmd_ref_msg)


    return
  

  def ctrCommandLoopTimerCallback(self, timer_msg):

    # Get time
    time_stamp_current = rospy.Time.now()

    #
    self.path_follower.pathFollowerLoop(time_stamp_current)

    # Publish
    if(self.path_follower.flag_set_robot_pose_ref):
      self.robotPoseRefPublish(time_stamp_current)

    if(self.path_follower.flag_set_robot_velo_world_ref):
      self.robotVelocityRefPublish(time_stamp_current)

    if(self.path_follower.flag_set_robot_velo_cmd_ref):
      self.robotVelCmdRefPublish(time_stamp_current)
    
    # End
    return

  