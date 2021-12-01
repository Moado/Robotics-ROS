#!/usr/bin/env python

import numpy as np
from numpy import *

import os


import rospy


from ars_path_planner_ros import *




def main():

  ars_path_planner_ros = ArsPathPlannerRos()

  ars_path_planner_ros.init()
  ars_path_planner_ros.open()

  try:
    ars_path_planner_ros.run()
  except rospy.ROSInterruptException:
    pass


  return 0



''' MAIN '''
if __name__ == '__main__':

  main()