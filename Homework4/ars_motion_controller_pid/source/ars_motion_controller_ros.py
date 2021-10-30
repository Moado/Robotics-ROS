#!/usr/bin/env python

import numpy as np
from numpy import *

import os




# ROS

import rospy

import rospkg

from std_msgs.msg import Header

import geometry_msgs.msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

import tf_conversions

import tf2_ros




#
from ars_motion_controller import *


#
import ars_lib_helpers





class ArsMotionControllerRos:

  #######


  # Robot frame
  robot_frame = 'robot_base_link'

  # World frame
  world_frame = 'world'



  # Vel loop freq 
  # time step
  vel_loop_freq = 50.0
  # Timer
  vel_loop_timer = None


  # Pos loop freq 
  # time step
  pos_loop_freq = 10.0
  # Timer
  pos_loop_timer = None



  # Robot command publisher
  flag_pub_robot_vel_cmd_unstamped = False
  robot_vel_cmd_unstamped_pub = None
  robot_vel_cmd_stamped_pub = None

  # Robot pose subscriber
  robot_pose_sub = None
  # Robot velocity subscriber
  robot_vel_world_sub = None

  # Robot pose subscriber
  robot_pose_ref_sub = None
  # Robot velocity subscriber
  robot_vel_world_ref_sub = None
  #
  robot_vel_cmd_ref_sub = None


  # Motion controller
  motion_controller = ArsMotionController()
  


  #########

  def __init__(self):

    return


  def init(self, node_name='ars_motion_controller_node'):
    #

    # Init ROS
    rospy.init_node(node_name, anonymous=True)
    # signal.signal(signal.SIGINT, self.cleanup)
    
    # Package path
    # fixed
    pkg_path = rospkg.RosPack().get_path('ars_motion_controller_pid')
    

    #### READING PARAMETERS ###
    
    # TODO

    ###

    
    # End
    return


  def open(self):

    # Subscriber
    # 
    self.robot_pose_sub = rospy.Subscriber('robot_pose', PoseStamped, self.robotPoseCallback)
    #
    self.robot_vel_world_sub = rospy.Subscriber('robot_velocity_world', TwistStamped, self.robotVelWorldCallback)
    
    # 
    self.robot_pose_ref_sub = rospy.Subscriber('robot_pose_ref', PoseStamped, self.robotPoseRefCallback)
    #
    self.robot_vel_world_ref_sub = rospy.Subscriber('robot_velocity_world_ref', TwistStamped, self.robotVelWorldRefCallback)
    #
    self.robot_vel_cmd_ref_sub = rospy.Subscriber('robot_cmd_ref', TwistStamped, self.robotCmdRefCallback)


    # Publisher
    # Robot cmd stamped
    self.robot_vel_cmd_stamped_pub = rospy.Publisher('robot_cmd_ctr_stamped', TwistStamped, queue_size=1)
    # Robot cmd unstamped
    if(self.flag_pub_robot_vel_cmd_unstamped):
      self.robot_vel_cmd_unstamped_pub = rospy.Publisher('robot_ctr_cmd', Twist, queue_size=1)



    # Timers
    #
    self.vel_loop_timer = rospy.Timer(rospy.Duration(1.0/self.vel_loop_freq), self.velLoopTimerCallback)
    #
    self.pos_loop_timer = rospy.Timer(rospy.Duration(1.0/self.pos_loop_freq), self.posLoopTimerCallback)


    # End
    return


  def run(self):

    rospy.spin()

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
    self.motion_controller.setRobotPose(robot_posi, robot_atti_quat_simp)

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
    self.motion_controller.setRobotVelWorld(lin_vel_world, ang_vel_world)

    #
    return


  def robotPoseRefCallback(self, robot_pose_ref_msg):

    # Position
    robot_posi_ref = np.zeros((3,), dtype=float)
    robot_posi_ref[0] = robot_pose_ref_msg.pose.position.x
    robot_posi_ref[1] = robot_pose_ref_msg.pose.position.y
    robot_posi_ref[2] = robot_pose_ref_msg.pose.position.z

    # Attitude quat simp
    robot_atti_quat_ref = ars_lib_helpers.Quaternion.zerosQuat()
    robot_atti_quat_ref[0] = robot_pose_ref_msg.pose.orientation.w
    robot_atti_quat_ref[1] = robot_pose_ref_msg.pose.orientation.x
    robot_atti_quat_ref[2] = robot_pose_ref_msg.pose.orientation.y
    robot_atti_quat_ref[3] = robot_pose_ref_msg.pose.orientation.z

    robot_atti_quat_simp_ref = ars_lib_helpers.Quaternion.getSimplifiedQuatRobotAtti(robot_atti_quat_ref)

    #
    self.motion_controller.setRobotPosRef(robot_posi_ref, robot_atti_quat_simp_ref)

    #
    return


  def robotVelWorldRefCallback(self, robot_vel_ref_msg):

    # Linear
    lin_vel_world_ref = np.zeros((3,), dtype=float)
    lin_vel_world_ref[0] = robot_vel_ref_msg.twist.linear.x
    lin_vel_world_ref[1] = robot_vel_ref_msg.twist.linear.y
    lin_vel_world_ref[2] = robot_vel_ref_msg.twist.linear.z

    # Angular
    ang_vel_world_ref = np.zeros((1,), dtype=float)
    ang_vel_world_ref[0] = robot_vel_ref_msg.twist.angular.z

    #
    self.motion_controller.setRobotVelWorldRef(lin_vel_world_ref, ang_vel_world_ref)

    #
    return


  def robotCmdRefCallback(self, robot_cmd_ref_msg):

    # Linear
    lin_vel_cmd_ref = np.zeros((3,), dtype=float)
    lin_vel_cmd_ref[0] = robot_cmd_ref_msg.twist.linear.x
    lin_vel_cmd_ref[1] = robot_cmd_ref_msg.twist.linear.y
    lin_vel_cmd_ref[2] = robot_cmd_ref_msg.twist.linear.z

    # Angular
    ang_vel_cmd_ref = np.zeros((1,), dtype=float)
    ang_vel_cmd_ref[0] = robot_cmd_ref_msg.twist.angular.z

    #
    self.motion_controller.setRobotVelCmdRef(lin_vel_cmd_ref, ang_vel_cmd_ref)

    #
    return


  def robotCmdPub(self):

    # Get
    robot_velo_cmd_time_stamp = self.motion_controller.getRobotVeloCmdTimeStamp()
    robot_velo_lin_cmd = self.motion_controller.getRobotVeloLinCmd()
    robot_velo_ang_cmd = self.motion_controller.getRobotVeloAngCmd()

    #
    robot_velo_cmd_stamped_msg = TwistStamped()

    robot_velo_cmd_stamped_msg.header.stamp = robot_velo_cmd_time_stamp
    robot_velo_cmd_stamped_msg.header.frame_id = self.robot_frame

    robot_velo_cmd_stamped_msg.twist.linear.x = robot_velo_lin_cmd[0]
    robot_velo_cmd_stamped_msg.twist.linear.y = robot_velo_lin_cmd[1]
    robot_velo_cmd_stamped_msg.twist.linear.z = robot_velo_lin_cmd[2]

    robot_velo_cmd_stamped_msg.twist.angular.x = 0.0
    robot_velo_cmd_stamped_msg.twist.angular.y = 0.0
    robot_velo_cmd_stamped_msg.twist.angular.z = robot_velo_ang_cmd[0]

    #
    if(self.robot_vel_cmd_stamped_pub):
      self.robot_vel_cmd_stamped_pub.publish(robot_velo_cmd_stamped_msg)
    if(self.flag_pub_robot_vel_cmd_unstamped and self.robot_vel_cmd_unstamped_pub):
      self.robot_vel_cmd_unstamped_pub.publish(robot_velo_cmd_stamped_msg.twist)

    #
    return




  def velLoopTimerCallback(self, timer_msg):

    # Get time
    time_stamp_current = rospy.Time.now()

    #
    self.motion_controller.velLoopMotionController(time_stamp_current)

    # Publish
    self.robotCmdPub()
    
    # End
    return


  def posLoopTimerCallback(self, timer_msg):

    # Get time
    time_stamp_current = rospy.Time.now()

    #
    self.motion_controller.posLoopMotionController(time_stamp_current)

    
    # End
    return




  