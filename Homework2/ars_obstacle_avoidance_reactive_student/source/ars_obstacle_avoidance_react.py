#!/usr/bin/env python

import numpy as np
from numpy import *

import math

import os


# ROS

import rospy

import visualization_msgs.msg
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


#
import ars_lib_helpers





class ArsObstacleAvoidanceReact:

  #######

  # Config parameters
  # Robot size radius
  robot_size_radius = None
  distance_of_influence = None
  gain_avoidance = None


  # Input: References
  flag_set_robot_velo_cmd_raw = False
  robot_velo_cmd_raw_time_stamp = rospy.Time(0.0, 0.0)
  robot_velo_lin_cmd_raw = None
  robot_velo_ang_cmd_raw = None


  # Input: Obstacles detected
  obstacles_detected_msg = None


  # Output: Commands
  robot_velo_cmd_avoid_time_stamp = rospy.Time(0.0, 0.0)
  robot_velo_lin_cmd_avoid = None
  robot_velo_ang_cmd_avoid = None
  

 

  #########

  def __init__(self):

    # Config parameters
    # Robot size radius
    self.robot_size_radius = 0.3


    # Input: References
    self.flag_set_robot_velo_cmd_raw = False
    self.robot_velo_cmd_raw_time_stamp = rospy.Time(0.0, 0.0)
    self.robot_velo_lin_cmd_raw = None
    self.robot_velo_ang_cmd_raw = None

    # Input: Obstacles detected
    self.obstacles_detected_msg = MarkerArray()

    # Output: Commands
    self.robot_velo_cmd_avoid_time_stamp = rospy.Time(0.0, 0.0)
    self.robot_velo_lin_cmd_avoid = np.zeros((3,), dtype=float)
    self.robot_velo_ang_cmd_avoid = np.zeros((1,), dtype=float)

    # End
    return


  def setRobotVeloCmdRaw(self, robot_velo_cmd_raw_time_stamp, robot_velo_lin_cmd_raw, robot_velo_ang_cmd_raw):

    self.flag_set_robot_velo_cmd_raw = True

    self.robot_velo_cmd_raw_time_stamp = robot_velo_cmd_raw_time_stamp

    self.robot_velo_lin_cmd_raw = robot_velo_lin_cmd_raw
    self.robot_velo_ang_cmd_raw = robot_velo_ang_cmd_raw

    #
    return


  def getRobotVeloCmdAvoidTimeStamp(self):
    return self.robot_velo_cmd_avoid_time_stamp

  def getRobotVeloLinCmdAvoid(self):
    return self.robot_velo_lin_cmd_avoid

  def getRobotVeloAngCmdAvoid(self):
    return self.robot_velo_ang_cmd_avoid


  def ctrLoopObstacleAvoidanceReact(self, time_stamp_ros):

    # Time stamp
    self.robot_velo_cmd_avoid_time_stamp = time_stamp_ros


    # Check
    if(self.flag_set_robot_velo_cmd_raw == False):
      # Set to zero
      self.robot_velo_lin_cmd_avoid = np.zeros((3,), dtype=float)
      self.robot_velo_ang_cmd_avoid = np.zeros((1,), dtype=float)
      #
      return

    # Initialize with the raw value
    self.robot_velo_lin_cmd_avoid[:] = self.robot_velo_lin_cmd_raw
    self.robot_velo_ang_cmd_avoid[:] = self.robot_velo_ang_cmd_raw

    # Iterate for all obstacles
    for obst_detected_i in self.obstacles_detected_msg.markers:

      # Size of obst i
      rad_obst_i = obst_detected_i.scale.x/2.0

      # Position of obst i wrt robot
      posi_2d_obst_i_wrt_robot = np.zeros((2,), dtype=float)
      posi_2d_obst_i_wrt_robot[0] = obst_detected_i.pose.position.x
      posi_2d_obst_i_wrt_robot[1] = obst_detected_i.pose.position.y


      ###### TODO STUDENT
      
      # Useful variable
      # self.robot_size_radius
      self.robot_size_radius = 0.3  # (r_robot)
      self.distance_of_influence = 1.25 # (d_inf)
      self.gain_avoidance = 2.5  # (K)


      distance = []

      d_i = sqrt((posi_2d_obst_i_wrt_robot[0]**2)+ (posi_2d_obst_i_wrt_robot[1]**2))

      d_ef_i = d_i - (rad_obst_i + self.robot_size_radius)
      

      if d_ef_i >= self.distance_of_influence:     
          u_rep_i = 0
          self.robot_velo_lin_cmd_avoid[0] += ((-posi_2d_obst_i_wrt_robot[0]/d_i)*u_rep_i)
          self.robot_velo_lin_cmd_avoid[1] += ((-posi_2d_obst_i_wrt_robot[1]/d_i)*u_rep_i)

      elif 0 < d_ef_i < self.distance_of_influence:   

          u_rep_i = - self.gain_avoidance * log(d_ef_i/self.distance_of_influence)         
          self.robot_velo_lin_cmd_avoid[0] += ((-posi_2d_obst_i_wrt_robot[0]/d_i)*u_rep_i)
          self.robot_velo_lin_cmd_avoid[1] += ((-posi_2d_obst_i_wrt_robot[1]/d_i)*u_rep_i)
      
      else: #infinity
          u_rep_i = float('inf')
          self.robot_velo_lin_cmd_avoid[0] = 50 #float('inf')
          self.robot_velo_lin_cmd_avoid[1] = 50 #float('inf')

      ###### END TODO BY STUDENT

      distance += d_i

    # End
    return
