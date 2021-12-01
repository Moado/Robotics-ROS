#!/usr/bin/env python

import numpy as np
from numpy import *

import math

import sys

import os

import time


# ROS

import rospy

import visualization_msgs.msg
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


#
import ars_lib_helpers

from ars_connected_graph import *
from ars_discrete_search import *



class ArsPathPlanner:

  #######


  # Input: Pose Feedback
  flag_set_robot_pose = False
  robot_posi = None
  robot_atti_quat_simp = None

  # Input: Obstacles environment
  flag_set_obstacles_world = False
  obstacles_world = MarkerArray()

  # Input: Pose Ref
  flag_set_robot_pose_ref = False
  robot_posi_ref = None
  robot_atti_quat_simp_ref = None

  # Output: Trajectory reference
  flag_set_robot_traj = False
  robot_traj = None
  robot_traj_raw = None


  # Roadmap
  roadmap = None

  # A-star
  a_star = None


  # Config parameters

  # environment dimensions
  env_dims = {}

  # Environment sampling distance maximum per dimension
  env_sampling_dist_max_dim = None

  # Nodes min neighbourhood
  min_neighbourhood = None
  
  # Robot size radius
  robot_size_radius = None
  #
  dist_influence = None
  #
  gain_avoidance = None

  # sampling distance cost calculation (in m)
  cost_sampling_dist_max = None

  cost = 0.0


  #########

  def __init__(self):

    # Input: Pose Feedback
    self.flag_set_robot_pose = False
    self.robot_posi = np.zeros((3,), dtype=float)
    self.robot_atti_quat_simp = ars_lib_helpers.Quaternion.zerosQuatSimp()

    # Input: Obstacles environment
    self.flag_set_obstacles_world = False
    self.obstacles_world = MarkerArray()

    # Input: Pose Referemce
    self.flag_set_robot_pose_ref = False
    self.robot_posi_ref = np.zeros((3,), dtype=float)
    self.robot_atti_quat_simp_ref = ars_lib_helpers.Quaternion.zerosQuatSimp()
    
    # Output Trajectory
    self.flag_set_robot_traj = False
    self.robot_traj = []
    self.robot_traj_raw = []


    # Roadmap
    self.roadmap = ConnectedUndirectedGeometricGraph()


    # A-star
    self.a_star = ArsDiscreteSearch()
    self.a_star.roadmap = self.roadmap
    #
    self.a_star.computeCostNode = self.computeCostNodeById
    self.a_star.computeCostG = self.computeCostWithObstBetweenNodesById
    self.a_star.computeCostH = self.computeDistanceBetweenNodesById


    # Config parameters


    # environment dimensions (in m)
    # self.env_dims = {'x': [-10.0, 10.0],
                      #'y': [-10.0, 10.0]}
    self.env_dims = {'x': [-2.0, 8.0],
                      'y': [-2.0, 8.0]}

    # Environment sampling distance maximum per dimension (in m)
    self.env_sampling_dist_max_dim = 0.5

    # Nodes min neighbourhood
    self.min_neighbourhood = 4

    # Robot size radius (in m)
    self.robot_size_radius = 0.3
    # (in m)
    self.dist_influence =  2 #1.25
    # 
    self.gain_avoidance =  3 #2.5

    # sampling distance cost calculation (in m)
    self.cost_sampling_dist_max = 0.5


    # End
    return


  def setRobotPose(self, robot_posi, robot_atti_quat_simp):
    
    self.flag_set_robot_pose = True

    self.robot_posi = robot_posi
    self.robot_atti_quat_simp = robot_atti_quat_simp

    return


  def setRobotPoseRef(self, robot_posi_ref, robot_atti_quat_simp_ref):
    
    self.flag_set_robot_pose_ref = True

    self.robot_posi_ref = robot_posi_ref
    self.robot_atti_quat_simp_ref = robot_atti_quat_simp_ref

    return


  def setObstaclesWorld(self, obstacles_world):

    self.flag_set_obstacles_world = True
    self.obstacles_world = obstacles_world

    return


  def init(self):

    # Generate roadmap
    print("Graph generation: start")
    self.generateRoadmap()
    print("Graph generation: ended")

    # End
    return


  def generateRoadmap(self, flag_verbose=False):

    # Init
    if(flag_verbose):
      print("Creating graph")

    # Creating nodes
    if(flag_verbose):
      print("Creating nodes")

    #
    num_samples_x = round((self.env_dims['x'][1] - self.env_dims['x'][0])/self.env_sampling_dist_max_dim) + 1
    num_samples_y = round((self.env_dims['y'][1] - self.env_dims['y'][0])/self.env_sampling_dist_max_dim) + 1
    
    #
    x_vals = np.linspace(self.env_dims['x'][0], self.env_dims['x'][1], num=num_samples_x, endpoint=True)
    y_vals = np.linspace(self.env_dims['y'][0], self.env_dims['y'][1], num=num_samples_y, endpoint=True)

    #
    for x in x_vals:
      for y in y_vals:
        self.roadmap.addNode(np.array([x, y]))

    # Connecting nodes
    if(flag_verbose):
      print("Connecting nodes")

    #
    self.roadmap.connectAllNodes(self.min_neighbourhood)

    # End
    if(flag_verbose):
      print("Graph created")

    self.roadmap.printGraph()

    # End
    return


  def computeCostObstaclesAtPosition(self, position_world):

    # Init
    cost = 0.0

    self.flag_set_obstacles_world = True

    # Iterate for all obstacles
    for obst_world_i in self.obstacles_world.markers:

      # Size of obst i
      rad_obst_i = obst_world_i.scale.x/2.0

      # Position of obst i wrt world
      posi_2d_obst_i_wrt_world = np.zeros((2,), dtype=float)
      posi_2d_obst_i_wrt_world[0] = obst_world_i.pose.position.x
      posi_2d_obst_i_wrt_world[1] = obst_world_i.pose.position.y
      

      ###### TODO BY STUDENT
      # Use:
      # Tunning parameters:
      # self.robot_size_radius
      # self.dist_influence
      # self.gain_avoidance
      # Useful Variables:
      # posi_2d_obst_i_wrt_world
      # position_world
      # rad_obst_i
      # Fill:
      # cost

      # Iterate for all obstacles


      ###### TODO STUDENT
      
      # Useful variable
      # self.robot_size_radius
      self.robot_size_radius = 0.3  # (r_robot)
      self.dist_influence = 1.25 # (d_inf)
      self.gain_avoidance = 2.5  # (K)



      d_i = math.sqrt((posi_2d_obst_i_wrt_world[0]-position_world[0])**2 + (posi_2d_obst_i_wrt_world[1]-position_world[1])**2)

      d_ef_i = d_i - (rad_obst_i + self.robot_size_radius)
      
      

      if d_ef_i >= self.dist_influence: 
          
          u_rep_i = 0
          
              
          
      elif d_ef_i < 0:

          u_rep_i = float('inf')
          
            

      else: 

          u_rep_i = -self.gain_avoidance * math.log10(d_ef_i/self.dist_influence)  

      ###### END TODO BY STUDENT
        
      
      
      cost += abs(u_rep_i)


    return cost


  def computeCostWithObstBetweenTwoPosi(self, position1, position2):

    # Init
    cost_total = 0.0

    # Sample the line between two points
    diff_posi_vec = position2 - position1
    diff_posi_dist = np.linalg.norm(diff_posi_vec)
    if(diff_posi_dist>0.001):
      diff_posi_vec_norm = diff_posi_vec/diff_posi_dist
    else:
      diff_posi_vec_norm = np.zeros((2,), dtype=float)

    num_samples_diff_posi_dist = round((diff_posi_dist - 0.0)/self.cost_sampling_dist_max) + 1

    sampl_diff_posi_dist_vals = np.linspace(0.0, diff_posi_dist, num=num_samples_diff_posi_dist, endpoint=True)

    sampl_diff_posi_vals = []

    for sampl_diff_posi_dist_vals_i in sampl_diff_posi_dist_vals:
      posi_i = position1 + diff_posi_vec_norm*sampl_diff_posi_dist_vals_i
      sampl_diff_posi_vals.append(posi_i)

    # Cost computation

    # Init
    cost_posi_i = self.computeCostObstaclesAtPosition(sampl_diff_posi_vals[0])

    # Loop
    for posi_i_idx in range(len(sampl_diff_posi_vals)-1):

      cost_posi_i_next = self.computeCostObstaclesAtPosition(sampl_diff_posi_vals[posi_i_idx+1])

      if(math.isinf(cost_posi_i) or math.isinf(cost_posi_i_next)):
        if(sys.version_info[0] < 3):
          cost_total = float('inf')
        else:
          cost_total = math.inf
        return cost_total

      dist_posi_i_i_next = np.linalg.norm(sampl_diff_posi_vals[posi_i_idx+1] - sampl_diff_posi_vals[posi_i_idx])

      cost_i = math.sqrt(dist_posi_i_i_next**2 + (cost_posi_i_next-cost_posi_i)**2)
      cost_total += cost_i

      # Check if infinite
      if(math.isinf(cost_total)):
        return cost_total

      # update for next iteration
      cost_posi_i = cost_posi_i_next

    # End
    return cost_total


  def computeCostNodeById(self, node_id):

    posi_node = self.roadmap.getNodePositionById(node_id)

    return self.computeCostObstaclesAtPosition(posi_node)


  def computeCostWithObstBetweenNodesById(self, node1_id, node2_id):

    posi_node1 = self.roadmap.getNodePositionById(node1_id)
    posi_node2 = self.roadmap.getNodePositionById(node2_id)

    return self.computeCostWithObstBetweenTwoPosi(posi_node1, posi_node2)


  def computeDistanceBetweenNodesById(self, node1_id, node2_id):

    posi_node1 = self.roadmap.getNodePositionById(node1_id)
    posi_node2 = self.roadmap.getNodePositionById(node2_id)

    distance = np.linalg.norm(posi_node1-posi_node2)

    return distance


  def shortenStateSolution(self):

    # Init
    self.states_solution_short = []

    # Check
    if(len(self.states_solution)==0):
      return

    # Compute cost raw
    cost_raw_vec = []
    for states_sol_idx in range(len(self.states_solution)-1):
      cost_raw_vec.append(self.computeCostWithObstBetweenNodesById(self.states_solution[states_sol_idx], self.states_solution[states_sol_idx+1]))

    # Start algorithm
    states_sol_i_idx = 0
    self.states_solution_short.append(self.states_solution[0])

    # Shortening algo
    while(self.states_solution_short[-1] != self.states_solution[-1]):

      for states_sol_j_idx in range(len(self.states_solution)-1, states_sol_i_idx, -1):

        # Cost short
        cost_short = self.computeCostWithObstBetweenNodesById(self.states_solution[states_sol_i_idx], self.states_solution[states_sol_j_idx])

        # Cost raw
        cost_raw = 0
        for states_sol_k_idx in range(states_sol_i_idx, states_sol_j_idx):
          cost_raw += cost_raw_vec[states_sol_k_idx]

        # Comparison
        if(cost_short<=cost_raw):
          self.states_solution_short.append(self.states_solution[states_sol_j_idx])
          break

      # Update for next iter
      states_sol_i_idx = states_sol_j_idx

    # End
    return


  def create3DPathFromSolution(self, states_solution):

    # Output
    robot_traj = []

    # Robot path in 2D
    robot_path = []
    for states_solution_i in states_solution:
      waypoint_i = self.roadmap.getNodePositionById(states_solution_i)
      robot_path.append(waypoint_i)

    # Compute total of path and distribute it
    length_path_acumm = []
    length_path_acumm.append(0.0)
    if(len(robot_path)!=0):
      for waypoint_path_i_idx in range(len(robot_path)-1):
        length_path_sect = np.linalg.norm(robot_path[waypoint_path_i_idx]-robot_path[waypoint_path_i_idx+1])
        length_path_acumm_i = length_path_acumm[-1] + length_path_sect
        length_path_acumm.append(length_path_acumm_i)
      perc_length_path_acumm =  [x / length_path_acumm[-1] for x in length_path_acumm]
    else:
      perc_length_path_acumm = 1.0

    # Delta pos z
    delta_pos_z = self.robot_posi_ref[2] - self.robot_posi[2]

    # Delta quat
    delta_ang = ars_lib_helpers.Quaternion.angleDiffFromQuatSimp(self.robot_atti_quat_simp_ref, self.robot_atti_quat_simp)

    # Robot path (all degrees of freedom)
    for waypoint_path_i_idx in range(len(robot_path)):
      waypoint_traj_i = ars_lib_helpers.PoseSimp()

      waypoint_traj_i.position[0:2] = robot_path[waypoint_path_i_idx]
      waypoint_traj_i.position[2] = self.robot_posi[2] + delta_pos_z * perc_length_path_acumm[waypoint_path_i_idx]

      delta_quat = ars_lib_helpers.Quaternion.quatSimpFromAngle( delta_ang * perc_length_path_acumm[waypoint_path_i_idx])
      waypoint_traj_i.attitude_quat_simp = ars_lib_helpers.Quaternion.quatSimpProd(self.robot_atti_quat_simp, delta_quat)

      robot_traj.append(waypoint_traj_i)

    # End
    return robot_traj


  def planPathCall(self, flag_verbose=True):

    if(flag_verbose):
      print('New path planning request')

    ### Reset variables
    self.flag_set_robot_traj = False
    self.robot_traj = []

    ### Checks
    if(not self.flag_set_robot_pose):
      print('Robot pose not set')
      return

    if(not self.flag_set_obstacles_world):
      print('Obstacle world not set')
      return

    if(not self.flag_set_robot_pose_ref):
      print('Robot pose ref not set')
      return

    ### Connect init and goal positions to the graph
    if(flag_verbose):
      start_time = time.time()

    # Init
    posi_node_init_in = self.robot_posi[0:2]
    node_init_id = self.roadmap.addAndConnectNode(posi_node_init_in, self.min_neighbourhood)

    # Goal
    posi_node_goal_in = self.robot_posi_ref[0:2]
    node_goal_id = self.roadmap.addAndConnectNode(posi_node_goal_in, self.min_neighbourhood)

    if(flag_verbose):
      end_time = time.time()
      print("--- Preparing graph: %s seconds ---" % (end_time - start_time))

    ### A-Star search - obtaining raw solution in graph
    if(flag_verbose):
      start_time = time.time()
    self.a_star.search(node_init_id, node_goal_id, flag_verbose=False)
    self.states_solution = self.a_star.states_solution
    if(flag_verbose):
      end_time = time.time()
      print("--- Search and plan A-Star: %s seconds ---" % (end_time - start_time))

    ### Shorten solution - obtaining shortened solution in graph
    if(flag_verbose):
      start_time = time.time()
    self.shortenStateSolution()
    if(flag_verbose):
      end_time = time.time()
      print("--- Shortening: %s seconds ---" % (end_time - start_time))

    ### Create robot path raw - obtaining raw solution in space
    self.robot_traj_raw = self.create3DPathFromSolution(self.states_solution)

    ###  Create robot path shorted - obtaining shortened solution in space
    self.robot_traj = self.create3DPathFromSolution(self.states_solution_short)

    ### Set flag
    self.flag_set_robot_traj = True

    ### Clean graph: Remove added nodes
    if(flag_verbose):
      start_time = time.time()
    self.roadmap.removeNode(node_init_id)
    self.roadmap.removeNode(node_goal_id)
    if(flag_verbose):
      end_time = time.time()
      print("--- Cleaning graph: %s seconds ---" % (end_time - start_time))

    ### End
    return
