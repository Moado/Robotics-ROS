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

import visualization_msgs.msg
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import tf_conversions

import tf2_ros




#
from ars_obstacle_avoidance_react import *


#
import ars_lib_helpers





class ArsObstacleAvoidanceReactRos:

  #######


  # Robot frame
  robot_frame = None


  # Ctr loop freq 
  # time step
  ctr_loop_freq = None
  # Timer
  ctr_loop_timer = None


  # Robot command raw subscriber
  robot_vel_cmd_raw_stamped_sub = None


  # Obstacles subscriber
  obstacles_detected_sub = None


  # Robot command avoidance publisher
  robot_vel_cmd_avoidance_stamped_pub = None



  # Obstacle avoidance reactive
  obstacle_avoidance_react = ArsObstacleAvoidanceReact()
  


  #########

  def __init__(self):

    # Robot frame
    self.robot_frame = 'robot_base_link'


    # Ctr loop freq 
    # time step
    self.ctr_loop_freq = 50.0
    # Timer
    self.ctr_loop_timer = None


    # Robot command raw subscriber
    self.robot_vel_cmd_raw_stamped_sub = None


    # Obstacles subscriber
    self.obstacles_detected_sub = None


    # Robot command avoidance publisher
    self.robot_vel_cmd_avoidance_stamped_pub = None

    #
    return


  def init(self, node_name='ars_obstacle_avoidance_react_node'):
    #

    # Init ROS
    rospy.init_node(node_name, anonymous=True)

    
    # Package path
    pkg_path = rospkg.RosPack().get_path('ars_obstacle_avoidance_react')
    

    #### READING PARAMETERS ###
    
    # TODO

    ###

    
    # End
    return


  def open(self):

    # Subscriber
    #
    self.robot_vel_cmd_raw_stamped_sub = rospy.Subscriber('robot_cmd_raw_stamped', TwistStamped, self.robotVelCmdRawStampedCallback)
    #
    self.obstacles_detected_sub = rospy.Subscriber('obstacles_detected', MarkerArray, self.obstaclesDetectedCallback)
    

    # Publisher
    # Robot cmd stamped
    self.robot_vel_cmd_avoidance_stamped_pub = rospy.Publisher('robot_cmd_avoidance_stamped', TwistStamped, queue_size=1)


    # Timers
    #
    self.ctr_loop_timer = rospy.Timer(rospy.Duration(1.0/self.ctr_loop_freq), self.ctrLoopTimerCallback)


    # End
    return


  def run(self):

    rospy.spin()

    return


  def robotVelCmdRawStampedCallback(self, robot_vel_cmd_raw_stamped_msg):

    # Timestamp
    robot_velo_cmd_raw_time_stamp = robot_vel_cmd_raw_stamped_msg.header.stamp

    # Baselink
    self.robot_frame = robot_vel_cmd_raw_stamped_msg.header.frame_id

    # Linear
    lin_vel_cmd_raw_ref = np.zeros((3,), dtype=float)
    lin_vel_cmd_raw_ref[0] = robot_vel_cmd_raw_stamped_msg.twist.linear.x
    lin_vel_cmd_raw_ref[1] = robot_vel_cmd_raw_stamped_msg.twist.linear.y
    lin_vel_cmd_raw_ref[2] = robot_vel_cmd_raw_stamped_msg.twist.linear.z

    # Angular
    ang_vel_cmd_raw_ref = np.zeros((1,), dtype=float)
    ang_vel_cmd_raw_ref[0] = robot_vel_cmd_raw_stamped_msg.twist.angular.z

    #
    self.obstacle_avoidance_react.setRobotVeloCmdRaw(robot_velo_cmd_raw_time_stamp, lin_vel_cmd_raw_ref, ang_vel_cmd_raw_ref)

    #
    return


  def obstaclesDetectedCallback(self, obstacles_detected_msg):
    
    # Save
    self.obstacle_avoidance_react.obstacles_detected_msg = obstacles_detected_msg

    #
    return


  def robotVelCmdAvoidancePub(self):

    # Get
    robot_velo_cmd_time_stamp = self.obstacle_avoidance_react.getRobotVeloCmdAvoidTimeStamp()
    robot_velo_lin_cmd = self.obstacle_avoidance_react.getRobotVeloLinCmdAvoid()
    robot_velo_ang_cmd = self.obstacle_avoidance_react.getRobotVeloAngCmdAvoid()

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
    if(self.robot_vel_cmd_avoidance_stamped_pub):
      self.robot_vel_cmd_avoidance_stamped_pub.publish(robot_velo_cmd_stamped_msg)

    #
    return


  def ctrLoopTimerCallback(self, timer_msg):

    # Get time
    time_stamp_current = rospy.Time.now()

    #
    self.obstacle_avoidance_react.ctrLoopObstacleAvoidanceReact(time_stamp_current)

    # Publish
    self.robotVelCmdAvoidancePub()
    
    # End
    return


  