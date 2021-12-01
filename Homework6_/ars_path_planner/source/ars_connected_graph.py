#!/usr/bin/env python

import numpy as np
from numpy import *



#
import ars_lib_helpers




###########################
# class GraphNode
###########################
class GraphNode:

  # Position
  position = np.zeros((2,), dtype=float)

  # List of connected edges ids
  list_edges_ids = []


  def __init__(self):

    # Position
    self.position = np.zeros((2,), dtype=float)

    # List of connected edges ids
    self.list_edges_ids = []

    return

  def __init__(self, position):
    # Position
    self.position = position

    # List of connected edges ids
    self.list_edges_ids = []

    return



###########################
# class GraphEdge
###########################
class GraphEdge:

  #
  node1 = None

  #
  node2 = None

  # distance
  distance = None

  def __init__(self):
    #
    self.node1 = None

    #
    self.node2 = None

    # distance
    self.distance = -1.0

    #
    return

  def __init__(self, node1, node2, distance = -1.0):
    self.node1 = node1
    self.node2 = node2
    self.distance = distance
    return
      



###########################
# class ConnectedUndirectedGeometricGraph
###########################
class ConnectedUndirectedGeometricGraph:

  #
  dict_graph_nodes = {}
  last_id_nodes = -1

  #
  dict_graph_edges = {}
  last_id_edges = -1



  def __init__(self):

    #
    self.dict_graph_nodes = {}
    self.last_id_nodes = -1

    #
    self.ict_graph_edges = {}
    self.last_id_edges = -1

    return


  def addNode(self, position):

    self.last_id_nodes += 1

    self.dict_graph_nodes[self.last_id_nodes] = GraphNode(np.array(position))

    return self.last_id_nodes


  def addEdge(self, node1_id, node2_id):

    # check if nodes exist
    if(not node1_id in self.dict_graph_nodes):
      print("Node 1 does not exist")
      return
    if(not node2_id in self.dict_graph_nodes):
      print("Node 2 does not exist")
      return

    # check if edge already exists
    for key in self.dict_graph_edges:
      if( (self.dict_graph_edges[key].node1 == node1_id and self.dict_graph_edges[key].node2 == node2_id ) or
        (self.dict_graph_edges[key].node1 == node2_id and self.dict_graph_edges[key].node2 == node1_id ) ):
        return

    # Distance
    distance = self.computeDistanceBetweenNodes(node1_id, node2_id)

    # Add edge
    self.last_id_edges += 1

    self.dict_graph_edges[self.last_id_edges] = GraphEdge(node1_id, node2_id, distance)

    self.dict_graph_nodes[node1_id].list_edges_ids.append(self.last_id_edges)
    self.dict_graph_nodes[node2_id].list_edges_ids.append(self.last_id_edges)

    # End
    return


  def removeNode(self, node_id):

    list_edges_node_id = self.dict_graph_nodes[node_id].list_edges_ids

    # Remove edges from node dict
    for edges_id_i in list_edges_node_id:
      if(self.dict_graph_edges[edges_id_i].node1 == node_id):
        other_node = self.dict_graph_edges[edges_id_i].node2
      else:
        other_node = self.dict_graph_edges[edges_id_i].node1
      self.dict_graph_nodes[other_node].list_edges_ids.remove(edges_id_i)

    # Remove
    for edges_id_i in list_edges_node_id:
      del self.dict_graph_edges[edges_id_i]

    # Remove
    self.dict_graph_nodes[node_id].list_edges_ids = []
    del self.dict_graph_nodes[node_id]

    return


  def removeAllEdges(self):

    self.last_id_edges = -1

    self.dict_graph_edges = {}

    for index, (key, value) in enumerate(self.dict_graph_nodes.items()):
      value.list_edges_ids = []

    return


  def getNodeIdByValue(self, position):

    for index, (key, value) in enumerate(self.dict_graph_nodes.items()):
      if(np.array_equal(value.position, position)):
        return key

    return -1


  def addAndConnectNode(self, position, neighbourhood=4):

    # Add node
    node_id = self.addNode(position)

    # Connect node
    self.connectNode(node_id, neighbourhood)

    # end
    return node_id


  def getNodePositionById(self, node_id):

    return self.dict_graph_nodes[node_id].position


  def computeDistanceBetweenNodes(self, node1_id, node2_id):

    if(not node1_id in self.dict_graph_nodes):
      print("Node 1 does not exist")
      return
    if(not node2_id in self.dict_graph_nodes):
      print("Node 2 does not exist")
      return

    dist_vec = self.dict_graph_nodes[node1_id].position - self.dict_graph_nodes[node2_id].position

    dist_val = np.linalg.norm(dist_vec)

    return dist_val


  def computeNeighbourNodes(self, node1_id, neighbourhood=4):

    list_neighbour_nodes = []

    for index, (key, value) in enumerate(self.dict_graph_nodes.items()):

      # self
      if(node1_id == key):
        continue

      # distance
      distance = self.computeDistanceBetweenNodes(node1_id, key)

      # check list neighbours
      if(len(list_neighbour_nodes) < neighbourhood):
        # add
        list_neighbour_nodes.append((key, distance))
      else:
        # find the largest distance
        large_value_list_idx = 0
        for list_idx_i, list_val_i in enumerate(list_neighbour_nodes):
          if(list_neighbour_nodes[list_idx_i][1]>list_neighbour_nodes[large_value_list_idx][1]):
            large_value_list_idx = list_idx_i
        # replace if needed
        if(distance<list_neighbour_nodes[large_value_list_idx][1]):
          list_neighbour_nodes[large_value_list_idx] = (key, distance)

    # End
    return list_neighbour_nodes


  def connectNode(self, node_id, min_neighbourhood=4):

    list_neighbour_nodes = self.computeNeighbourNodes(node_id, min_neighbourhood)
    for list_idx_i, list_val_i in enumerate(list_neighbour_nodes):
      self.addEdge(node_id, list_neighbour_nodes[list_idx_i][0])

    return


  def connectAllNodes(self, min_neighbourhood=4):

    # remove all edges
    self.removeAllEdges()

    #
    for index, (key, value) in enumerate(self.dict_graph_nodes.items()):
      list_neighbour_nodes = self.computeNeighbourNodes(key, min_neighbourhood)
      for list_idx_i, list_val_i in enumerate(list_neighbour_nodes):
        self.addEdge(key, list_neighbour_nodes[list_idx_i][0])

    return


  def getConnectedNodes(self, node1_id, flag_get_distance=False):

    list_connected_nodes = []

    for edge_id_i in self.dict_graph_nodes[node1_id].list_edges_ids:
      if(not node1_id == self.dict_graph_edges[edge_id_i].node1):
        if(flag_get_distance):
          list_connected_nodes.append((self.dict_graph_edges[edge_id_i].node1,self.dict_graph_edges[edge_id_i].distance))
        else:
          list_connected_nodes.append(self.dict_graph_edges[edge_id_i].node1)
      else:
        if(flag_get_distance):
          list_connected_nodes.append((self.dict_graph_edges[edge_id_i].node2,self.dict_graph_edges[edge_id_i].distance))
        else:
          list_connected_nodes.append(self.dict_graph_edges[edge_id_i].node2)

    return list_connected_nodes


  def printGraph(self, flag_disp_nodes = True, flag_disp_edges = False):
    print("Graph")
    if(flag_disp_nodes):
      print("Nodes:")
      for key in self.dict_graph_nodes:
        print(key, '->', self.dict_graph_nodes[key].position, ';', self.dict_graph_nodes[key].list_edges_ids)

    if(flag_disp_edges):
      print("Edges:")
      for key in self.dict_graph_edges:
        print(key, '->', self.dict_graph_edges[key].node1, '; ', self.dict_graph_edges[key].node2, '; ',self.dict_graph_edges[key].distance)


  def getNumberOfNodes(self):
    return len(self.dict_graph_nodes)
