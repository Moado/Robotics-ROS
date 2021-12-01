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
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

import visualization_msgs.msg
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import nav_msgs.msg
from nav_msgs.msg import Path

import tf_conversions

import tf2_ros




#
from ars_path_planner import *


#
import ars_lib_helpers





class ArsPathPlannerRos:

  #######

  # World frame
  world_frame = 'world'

  # Timestamp reference
  time_stamp_reference = rospy.Time(0)

  # Robot traj publisher
  robot_traj_ref_pub = None
  robot_traj_ref_raw_pub = None

  # Robot pose subscriber
  robot_pose_sub = None

  # Obstacles world subscriber
  obstacles_world_sub = None

  # Robot pose ref subscriber
  robot_pose_ref_sub = None


  # Motion controller
  path_planner = ArsPathPlanner()
  


  #########

  def __init__(self):

    # Path planner
    self.path_planner = ArsPathPlanner()

    # end
    return


  def init(self, node_name='ars_path_planner_node'):
    #

    # Init ROS
    rospy.init_node(node_name, anonymous=True)

    
    # Package path
    pkg_path = rospkg.RosPack().get_path('ars_path_planner')
    

    #### READING PARAMETERS ###
    
    # TODO

    ###

    # init path planner
    self.path_planner.init()

    
    # End
    return


  def open(self):

    # Subscribers

    #
    self.robot_pose_ref_sub = rospy.Subscriber('robot_pose_des', PoseStamped, self.robotPoseRefCallback, queue_size=1)

    # 
    self.robot_pose_sub = rospy.Subscriber('robot_pose', PoseStamped, self.robotPoseCallback, queue_size=1)

    #
    self.obstacles_world_sub = rospy.Subscriber('obstacles_world', MarkerArray, self.obstaclesWorldCallback, queue_size=1)
    

    # Publishers

    # 
    self.robot_traj_ref_raw_pub = rospy.Publisher('robot_trajectory_ref_raw', Path, queue_size=1)
    # 
    self.robot_traj_ref_pub = rospy.Publisher('robot_trajectory_ref', Path, queue_size=1)


    # End
    return


  def run(self):

    rospy.spin()

    return


  def robotPoseRefCallback(self, robot_pose_ref_msg):

    # Timestamp
    if(robot_pose_ref_msg.header.stamp == rospy.Time(0)):
      self.time_stamp_reference = rospy.Time.now()
    else:
      self.time_stamp_reference = robot_pose_ref_msg.header.stamp

    # Transform message
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

    # Set reference in the path planner
    self.path_planner.setRobotPoseRef(robot_posi_ref, robot_atti_quat_simp_ref)

    # Call the path planner
    self.path_planner.planPathCall()

    # Publish planned path
    self.robotTrajRefPublish()

    #
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
    self.path_planner.setRobotPose(robot_posi, robot_atti_quat_simp)

    #
    return


  def obstaclesWorldCallback(self, obstacles_world_msg):

    obstacles_world_msg_filtered = MarkerArray()

    for obst_i_msg in obstacles_world_msg.markers:
      if(obst_i_msg.action == 0):
        obstacles_world_msg_filtered.markers.append(obst_i_msg)

    # Set
    self.path_planner.setObstaclesWorld(obstacles_world_msg_filtered)

    #
    return


  def robotTrajMsgGen(self, robot_traj):

    # Generate message
    robot_traj_ref_msg = Path()

    robot_traj_ref_msg.header.stamp = self.time_stamp_reference
    robot_traj_ref_msg.header.frame_id = self.world_frame

    robot_traj_ref_msg.poses = []

    if(self.path_planner.flag_set_robot_traj):
      for pose_i in robot_traj:

        pose_i_msg = PoseStamped()

        pose_i_msg.header.stamp = rospy.Time()
        pose_i_msg.header.frame_id = self.world_frame

        pose_i_msg.pose.position.x = pose_i.position[0]
        pose_i_msg.pose.position.y = pose_i.position[1]
        pose_i_msg.pose.position.z = pose_i.position[2]

        pose_i_msg.pose.orientation.w = pose_i.attitude_quat_simp[0]
        pose_i_msg.pose.orientation.x = 0.0
        pose_i_msg.pose.orientation.y = 0.0
        pose_i_msg.pose.orientation.z = pose_i.attitude_quat_simp[1]

        robot_traj_ref_msg.poses.append(pose_i_msg)

    return robot_traj_ref_msg


  def robotTrajRefPublish(self):

    #
    robot_traj_ref_msg = self.robotTrajMsgGen(self.path_planner.robot_traj)
    robot_traj_ref_raw_msg = self.robotTrajMsgGen(self.path_planner.robot_traj_raw)

    # Publish
    self.robot_traj_ref_pub.publish(robot_traj_ref_msg)
    self.robot_traj_ref_raw_pub.publish(robot_traj_ref_raw_msg)

    #
    return

