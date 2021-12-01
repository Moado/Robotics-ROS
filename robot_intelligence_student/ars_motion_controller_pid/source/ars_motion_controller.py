#!/usr/bin/env python

import numpy as np
from numpy import *

import os


# ROS

import rospy

import tf_conversions as tf


#
import ars_lib_helpers

#
import ars_pid




class ArsMotionController:

  #######

  # References
  #
  flag_set_robot_pose_ref = False
  robot_posi_ref = None
  robot_atti_quat_simp_ref = None
  #
  flag_set_robot_velo_world_ref = False
  robot_velo_lin_world_ref = None
  robot_velo_ang_world_ref = None
  #
  flag_set_robot_velo_cmd_ref = False
  # m/s
  robot_velo_lin_cmd_ref = None
  # rad/s
  robot_velo_ang_cmd_ref = None


  # Feedback
  #
  flag_set_robot_pose = False
  robot_posi = None
  robot_atti_quat_simp = None
  #
  flag_set_robot_vel_world = False
  robot_velo_lin_world = None
  robot_velo_ang_world = None


  # Commands
  robot_velo_cmd_time_stamp = rospy.Time(0.0, 0.0)
  robot_velo_lin_cmd = None
  robot_velo_ang_cmd = None
  

  # Loops Internal

  # Vel loop
  # Not needed!
  #
  #vel_loop_time_stamp_ros = rospy.Time(0.0, 0.0)
  #vel_loop_out_lin_cmd = None
  #vel_loop_out_ang_cmd = None
  
  # Pos loop
  #
  pos_loop_time_stamp_ros = rospy.Time(0.0, 0.0)
  flag_set_pos_loop_out = False
  pos_loop_out_lin_cmd = None
  pos_loop_out_ang_cmd = None


  # PIDs
  # Pose
  #
  flag_ctr_pos_hor = True
  pos_hor_pid = ars_pid.PID()
  #
  flag_ctr_pos_z = True
  pos_z_pid = ars_pid.PID()
  #
  flag_ctr_att_yaw = True
  att_yaw_pid = ars_pid.PID()

  # Velocity
  #
  flag_ctr_vel_lin_hor = True
  vel_lin_hor_pid = ars_pid.PID()
  #
  flag_ctr_vel_lin_z = True
  vel_lin_z_pid = ars_pid.PID()
  #
  flag_ctr_vel_ang_z = True
  vel_ang_z_pid = ars_pid.PID()




  #########

  def __init__(self):

    # Commands
    self.robot_velo_cmd_time_stamp = rospy.Time(0.0, 0.0)
    self.robot_velo_lin_cmd = np.zeros((3,), dtype=float)
    self.robot_velo_ang_cmd = np.zeros((1,), dtype=float)

    # Feedback
    #
    self.flag_set_robot_pose = False
    self.robot_posi = np.zeros((3,), dtype=float)
    self.robot_atti_quat_simp = ars_lib_helpers.Quaternion.zerosQuatSimp()
    #
    self.flag_set_robot_vel_world = False
    self.robot_velo_lin_world = np.zeros((3,), dtype=float)
    self.robot_velo_ang_world = np.zeros((1,), dtype=float)

    # References
    #
    self.flag_set_robot_pose_ref = False
    self.robot_posi_ref = np.zeros((3,), dtype=float)
    self.robot_atti_quat_simp_ref = ars_lib_helpers.Quaternion.zerosQuatSimp()
    #
    self.flag_set_robot_velo_world_ref = False
    self.robot_velo_lin_world_ref = np.zeros((3,), dtype=float)
    self.robot_velo_ang_world_ref = np.zeros((1,), dtype=float)
    #
    self.flag_set_robot_velo_cmd_ref = False
    self.robot_velo_lin_cmd_ref = np.zeros((3,), dtype=float)
    self.robot_velo_ang_cmd_ref = np.zeros((1,), dtype=float)

    # Internal
    # Vel loop
    # Not needed!
    #self.vel_loop_time_stamp_ros = rospy.Time(0.0, 0.0)
    #self.vel_loop_out_lin_cmd = np.zeros((3,1), dtype=float)
    #self.vel_loop_out_ang_cmd = np.zeros((1,1), dtype=float)
    # Pos loop
    self.pos_loop_time_stamp_ros = rospy.Time(0.0, 0.0)
    self.flag_set_pos_loop_out = False
    self.pos_loop_out_lin_cmd = np.zeros((3,), dtype=float)
    self.pos_loop_out_ang_cmd = np.zeros((1,), dtype=float)

    # PIDs
    # Pos
    #
    self.flag_ctr_pos_hor = True
    self.pos_hor_pid = ars_pid.PID()
    self.pos_hor_pid.setGainsPID(gain_P=1.0)
    self.pos_hor_pid.setAntiWindUp(-0.1, 0.1)
    self.pos_hor_pid.setCtrCmdSaturation(-5.0, 5.0)
    #
    self.flag_ctr_pos_z = True
    self.pos_z_pid = ars_pid.PID()
    self.pos_z_pid.setGainsPID(gain_P=1.0)
    self.pos_z_pid.setAntiWindUp(-0.1, 0.1)
    self.pos_z_pid.setCtrCmdSaturation(-5.0, 5.0)
    #
    self.flag_ctr_att_yaw = True
    self.att_yaw_pid = ars_pid.PID()
    self.att_yaw_pid.setGainsPID(gain_P=1.0)
    self.att_yaw_pid.setAntiWindUp(-0.1, 0.1)
    self.att_yaw_pid.setCtrCmdSaturation(-5.0, 5.0)

    # Vel
    #
    self.flag_ctr_vel_lin_hor = True
    self.vel_lin_hor_pid = ars_pid.PID()
    self.vel_lin_hor_pid.setGainsPID(gain_P=1.0)
    self.vel_lin_hor_pid.setAntiWindUp(-0.1, 0.1)
    self.vel_lin_hor_pid.setCtrCmdSaturation(-1.0, 1.0)
    #
    self.flag_ctr_vel_lin_z = True
    self.vel_lin_z_pid = ars_pid.PID()
    self.vel_lin_z_pid.setGainsPID(gain_P=1.0)
    self.vel_lin_z_pid.setAntiWindUp(-0.1, 0.1)
    self.vel_lin_z_pid.setCtrCmdSaturation(-1.0, 1.0)
    #
    self.flag_ctr_vel_ang_z = True
    self.vel_ang_z_pid = ars_pid.PID()
    self.vel_ang_z_pid.setGainsPID(gain_P=1.0)
    self.vel_ang_z_pid.setAntiWindUp(-0.1, 0.1)
    self.vel_ang_z_pid.setCtrCmdSaturation(-1.0, 1.0)

    # End
    return

  def setRobotPosRef(self, robot_posi_ref, robot_atti_quat_simp_ref):

    self.flag_set_robot_pose_ref = True

    self.robot_posi_ref = robot_posi_ref
    self.robot_atti_quat_simp_ref = robot_atti_quat_simp_ref

    return

  def setRobotVelWorldRef(self, lin_vel_world_ref, ang_vel_world_ref):

    self.flag_set_robot_velo_world_ref = True

    self.robot_velo_lin_world_ref = lin_vel_world_ref
    self.robot_velo_ang_world_ref = ang_vel_world_ref

    return

  def setRobotVelCmdRef(self, lin_vel_cmd_ref, ang_vel_cmd_ref):

    self.flag_set_robot_velo_cmd_ref = True

    self.robot_velo_lin_cmd_ref = lin_vel_cmd_ref
    self.robot_velo_ang_cmd_ref = ang_vel_cmd_ref

    return

  def setRobotPose(self, robot_posi, robot_atti_quat_simp):
    
    self.flag_set_robot_pose = True

    self.robot_posi = robot_posi
    self.robot_atti_quat_simp = robot_atti_quat_simp

    return

  def setRobotVelWorld(self, lin_vel_world, ang_vel_world):

    self.flag_set_robot_vel_world = True

    self.robot_velo_lin_world = lin_vel_world
    self.robot_velo_ang_world = ang_vel_world

    return

  def getRobotVeloCmdTimeStamp(self):
    return self.robot_velo_cmd_time_stamp

  def getRobotVeloLinCmd(self):
    return self.robot_velo_lin_cmd

  def getRobotVeloAngCmd(self):
    return self.robot_velo_ang_cmd


  def velLoopMotionController(self, time_stamp_ros):

    # Time stamp
    self.robot_velo_cmd_time_stamp = time_stamp_ros

    # Conversion (from world to robot)
    # Reference
    pos_loop_out_lin_cmd_robot = ars_lib_helpers.Conversions.convertVelLinFromWorldToRobot(self.pos_loop_out_lin_cmd, self.robot_atti_quat_simp)
    pos_loop_out_ang_cmd_robot = ars_lib_helpers.Conversions.convertVelAngFromWorldToRobot(self.pos_loop_out_ang_cmd, self.robot_atti_quat_simp)
    # Feedback
    robot_velo_lin_robot = ars_lib_helpers.Conversions.convertVelLinFromWorldToRobot(self.robot_velo_lin_world, self.robot_atti_quat_simp)
    robot_velo_ang_robot = ars_lib_helpers.Conversions.convertVelAngFromWorldToRobot(self.robot_velo_ang_world, self.robot_atti_quat_simp)

    # Initialization
    robot_velo_lin_cmd_ff = np.zeros((3,), dtype=float)
    robot_velo_lin_cmd_fb = np.zeros((3,), dtype=float)
    robot_velo_ang_cmd_ff = np.zeros((1,), dtype=float)
    robot_velo_ang_cmd_fb = np.zeros((1,), dtype=float)

    # Velocity Linear horizontal (x & y)
    # Feedforward
    # TODO by student
    # Use: self.robot_velo_lin_cmd_ref[0:2]
    robot_velo_lin_cmd_ff[0:2] = self.robot_velo_lin_cmd_ref[0:2]
    # Feedback
    if(self.flag_ctr_vel_lin_hor and self.flag_set_pos_loop_out and self.flag_set_robot_vel_world):
      
      # TODO by student
      # Use: pos_loop_out_lin_cmd_robot[0:2], robot_velo_lin_robot[0:2], time_stamp_ros, self.vel_lin_hor_pid
      
      #error = reference - feedback
      error_velo_lin_horizontal = pos_loop_out_lin_cmd_robot[0:2] - robot_velo_lin_robot[0:2]


      mod_error_velo_lin_horizontal = math.sqrt(error_velo_lin_horizontal[0]**2 + error_velo_lin_horizontal[1]**2) 
  
      if mod_error_velo_lin_horizontal != 0:

        #normalized_error  
        normalized_error_velo_lin_horizontal = error_velo_lin_horizontal / mod_error_velo_lin_horizontal
       
      else: 

        normalized_error_robot_posi = 0

      
      #output = normalized_error * ctr(mod_error)
      ctr_vel_lin = self.vel_lin_hor_pid.call(time_stamp_ros, mod_error_velo_lin_horizontal)
      robot_velo_lin_cmd_fb[0:2] = normalized_error_velo_lin_horizontal * ctr_vel_lin
    

    # Total
    # TODO by student
    # Use: robot_velo_lin_cmd_ff[0:2], robot_velo_lin_cmd_fb[0:2]
    self.robot_velo_lin_cmd[0:2] =  robot_velo_lin_cmd_ff[0:2] + robot_velo_lin_cmd_fb[0:2]

   

    # Velocity Linear vertical (z)
    # Feedforward
    # TODO by student
    # Use self.robot_velo_lin_cmd_ref[2]
    robot_velo_lin_cmd_ff[2] = self.robot_velo_lin_cmd_ref[2]
    # Feedback
    if(self.flag_ctr_vel_lin_z and self.flag_set_pos_loop_out and self.flag_set_robot_vel_world):
      # TODO by student
      # Use: pos_loop_out_lin_cmd_robot[2], robot_velo_lin_robot[2], time_stamp_ros, self.vel_lin_z_pid


      #error = reference - feedback
      error_velo_lin_vertical = pos_loop_out_lin_cmd_robot[2] - robot_velo_lin_robot[2]

      
      #output = error * ctr(mod_error)
      ctr_vel_lin = self.vel_lin_z_pid.call(time_stamp_ros, error_velo_lin_vertical)
      robot_velo_lin_cmd_fb[2] = ctr_vel_lin
    

    # Total
    # TODO by student
    # Use: robot_velo_lin_cmd_ff[2], robot_velo_lin_cmd_fb[2]
    self.robot_velo_lin_cmd[2] =  robot_velo_lin_cmd_ff[2] + robot_velo_lin_cmd_fb[2]



    # Velocity Angular (z)
    # Feedforward
    robot_velo_ang_cmd_ff[0] = self.robot_velo_ang_cmd_ref[0]
    # Feedback
    if(self.flag_ctr_vel_ang_z and self.flag_set_pos_loop_out and self.flag_set_robot_vel_world):
      error_vel_ang_z = pos_loop_out_ang_cmd_robot - robot_velo_ang_robot
      robot_velo_ang_cmd_fb[0] = self.vel_ang_z_pid.call(time_stamp_ros, error_vel_ang_z)
    # Total
    self.robot_velo_ang_cmd[0] = robot_velo_ang_cmd_ff[0] + robot_velo_ang_cmd_fb[0]

    # End
    return


  def posLoopMotionController(self, time_stamp_ros):

    # Time stamp
    self.pos_loop_time_stamp_ros = time_stamp_ros

    # Initialization
    pos_loop_out_lin_cmd_ff = np.zeros((3,), dtype=float)
    pos_loop_out_lin_cmd_fb = np.zeros((3,), dtype=float)
    pos_loop_out_ang_cmd_ff = np.zeros((1,), dtype=float)
    pos_loop_out_ang_cmd_fb = np.zeros((1,), dtype=float)

    # Linear horizontal (x & y)
    # Feedforward
    # TODO by student
    # Use: self.robot_velo_lin_world_ref[0:2]
    pos_loop_out_lin_cmd_ff[0:2] = self.robot_velo_lin_world_ref[0:2]
    # Feedback
    if(self.flag_ctr_pos_hor and self.flag_set_robot_pose and self.flag_set_robot_pose_ref):
      # TODO by student
      # Use: self.robot_posi_ref[0:2], self.robot_posi[0:2], time_stamp_ros, self.pos_hor_pid
      
      #error = reference - feedback
      error_robot_posi = self.robot_posi_ref[0:2] - self.robot_posi[0:2]

      
      mod_error_robot_posi = math.sqrt(error_robot_posi[0]**2 + error_robot_posi[1]**2) 
  
      if mod_error_robot_posi !=0:
        #normalized_error  
        normalized_error_robot_posi = error_robot_posi / mod_error_robot_posi 

      else:

        normalized_error_robot_posi = 0

      
      #output = normalized_error * ctr(mod_error)
      ctr_robot_posi = self.pos_hor_pid.call(time_stamp_ros, mod_error_robot_posi)
      pos_loop_out_lin_cmd_fb[0:2] = normalized_error_robot_posi * ctr_robot_posi
    

    # Total
    # TODO by student
    # Use: pos_loop_out_lin_cmd_ff[0:2], pos_loop_out_lin_cmd_fb[0:2]
    self.pos_loop_out_lin_cmd[0:2] = pos_loop_out_lin_cmd_ff[0:2] + pos_loop_out_lin_cmd_fb[0:2]

    # Linear vertical (z)
    # Feedforward
    # TODO by student
    # Use: self.robot_velo_lin_world_ref[2]
    pos_loop_out_lin_cmd_ff[2] = self.robot_velo_lin_world_ref[2]
    # Feedback
    if(self.flag_ctr_pos_z and self.flag_set_robot_pose and self.flag_set_robot_pose_ref):
      # TODO by student
      # Use: self.robot_posi_ref[2], self.robot_posi[2], time_stamp_ros, self.pos_z_pid
      
   
      #error = reference - feedback
      error_pos_loop = self.robot_posi_ref[2] - self.robot_posi[2]

      
      #output = error * ctr(mod_error)
      ctr_pos_loop = self.pos_z_pid.call(time_stamp_ros, error_pos_loop)
      pos_loop_out_lin_cmd_fb[2] = ctr_pos_loop


    # Total
    # TODO by student
    # Use: pos_loop_out_lin_cmd_ff[2], pos_loop_out_lin_cmd_fb[2]
    self.pos_loop_out_lin_cmd[2] = pos_loop_out_lin_cmd_ff[2] + pos_loop_out_lin_cmd_fb[2]




    # Angular (z)
    # Feedforward
    pos_loop_out_ang_cmd_ff[0] = self.robot_velo_ang_world_ref[0]
    # Feedback
    if(self.flag_ctr_att_yaw and self.flag_set_robot_pose and self.flag_set_robot_pose_ref):
      error_att_z = ars_lib_helpers.Quaternion.errorDiffFromQuatSimp(self.robot_atti_quat_simp_ref,self.robot_atti_quat_simp)
      pos_loop_out_ang_cmd_fb[0] = self.att_yaw_pid.call(time_stamp_ros, error_att_z)
    # Total
    self.pos_loop_out_ang_cmd[0] = pos_loop_out_ang_cmd_ff[0] + pos_loop_out_ang_cmd_fb[0]

    # Flag
    self.flag_set_pos_loop_out = True

    # End
    return
