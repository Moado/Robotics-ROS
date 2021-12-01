#!/usr/bin/env python

import numpy as np
from numpy import *

import os


import rospy


from ars_motion_controller_ros import *




def main():

  ars_motion_controller_ros = ArsMotionControllerRos()

  ars_motion_controller_ros.init()
  ars_motion_controller_ros.open()

  try:
    ars_motion_controller_ros.run()
  except rospy.ROSInterruptException:
    pass


  return 0



''' MAIN '''
if __name__ == '__main__':

  main()