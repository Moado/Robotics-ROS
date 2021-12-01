#!/usr/bin/env python

import numpy as np
from numpy import *

import os


import rospy


from ars_path_follower_ros import *




def main():

  ars_path_follower_ros = ArsPathFollowerRos()

  ars_path_follower_ros.init()
  ars_path_follower_ros.open()

  try:
    ars_path_follower_ros.run()
  except rospy.ROSInterruptException:
    pass


  return 0



''' MAIN '''
if __name__ == '__main__':

  main()