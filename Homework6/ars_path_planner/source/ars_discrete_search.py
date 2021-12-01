#!/usr/bin/env python

import numpy as np
from numpy import *

import math

import sys

import os


# ROS

import rospy

import visualization_msgs.msg
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from collections import deque


#
import ars_lib_helpers

from ars_connected_graph import *



###########################
# class ArsDiscreteSearch
###########################
class ArsDiscreteSearch:

  #######

  # Roadmap
  roadmap = None

  # 
  flag_path_found = False
  action_sequence = []
  states_solution = []
  closed_list = []
  
  # functions: f(n) = g(n) + h(n)
  computeCostNode = None
  computeCostG = None
  computeCostH = None



  #########

  def __init__(self):

    # Roadmap
    self.roadmap = None

    # 
    self.flag_path_found = False
    self.action_sequence = []
    self.states_solution = []
    self.closed_list = []
    
    # functions: f(n) = g(n) + h(n)
    self.computeCostNode = None
    self.computeCostG = None
    self.computeCostH = None

    # End
    return

  def reconstructPath(self,cameFrom,node_goal_id):
        
        path = deque()
        node = node_goal_id
        path.appendleft(node)

        self.states_solution = []
         

        while node in cameFrom:
            node = cameFrom[node]
            path.appendleft(node)

        self.states_solution = path
        self.flag_path_found = True
        
        return self.states_solution   
        

  def getLowest(self,openSet,fScore):
        lowest = float("inf")
        lowestNode = None
        for node in openSet:
            if fScore[node] < lowest:
                lowest = fScore[node]
                lowestNode = node
        return lowestNode  

  
  def searchInGraph(self, node_init_id, node_goal_id, flag_verbose=False):

        # Clear previous existing search
        self.flag_path_found = False

        self.action_sequence = []
        self.states_solution = []

        # Initialize both open and closed list
        


        ########## TODO BY STUDENT
          
        cameFrom = {}
        openSet = set([node_init_id])
        closedSet = set()
        gScore = {}
        fScore = {}
        gScore[node_init_id] = 0
        fScore[node_init_id] = gScore[node_init_id] + self.computeCostH(node_init_id,node_goal_id)
        while len(openSet) != 0:
            current = self.getLowest(openSet,fScore)
            if current == node_goal_id:
                return self.reconstructPath(cameFrom,node_goal_id)
            openSet.remove(current)
            closedSet.add(current)
            for neighbor in self.roadmap.getConnectedNodes(current):
                tentative_gScore = gScore[current] + self.computeCostG(current,neighbor)
                if neighbor in closedSet and tentative_gScore >= gScore[neighbor]:
                    continue
                if neighbor not in closedSet or tentative_gScore < gScore[neighbor]:
                    cameFrom[neighbor] = current
                    gScore[neighbor] = tentative_gScore
                    fScore[neighbor] = gScore[neighbor] + self.computeCostH(neighbor,node_goal_id)
                    if neighbor not in openSet:
                        openSet.add(neighbor)
        return self.states_solution



  def search(self, node_init_id, node_goal_id, flag_verbose=False):

    # Search in graph
    self.searchInGraph(node_init_id, node_goal_id, flag_verbose)


    return
