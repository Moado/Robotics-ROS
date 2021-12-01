#!/usr/bin/env python

import numpy as np
from numpy import *

import os


# ROS

import rospy

import tf_conversions as tf


#
import ars_lib_helpers



class PID:

  #######
  
  prev_time_stamp_ros = rospy.Time(0.0, 0.0)


  prev_error = 0.0
  
  error_integral = 0.0

  # Config parameters
  #
  gains = {'P': 1.0, 'D': 1.0, 'I': 1.0}  

  #
  anti_windup = {'MaxI': inf, 'MinI': -inf}
  #
  ctr_cmd_sat = {'Max': inf, 'Min': -inf}
  

  def __init__(self):

    self.prev_time_stamp_ros = rospy.Time(0.0, 0.0)
    self.gains = {'P': 1.5, 'D': 0.5, 'I': 0.001}


    self.prev_error = 0.0

    self.error_integral = 0.0

    self.sum_error = 0

    self.p = 0
    self.i = 0
    self.d = 0

    self.anti_windup = {'MaxI': inf, 'MinI': -inf}
    self.ctr_cmd_sat = {'Max': inf, 'Min': -inf}

    return


  def setGainsPID(self, gain_P=0.0, gain_I=0.0, gain_D=0.0):
    self.gains['P']=gain_P
    self.gains['I']=gain_I
    self.gains['D']=gain_D
    return

  def setAntiWindUp(self, anti_windup_min=-inf, anti_windup_max=inf):
    self.anti_windup['MaxI']=anti_windup_max
    self.anti_windup['MinI']=anti_windup_min
    return

  def setCtrCmdSaturation(self, ctr_cmd_min=-inf, ctr_cmd_max=inf):
    self.ctr_cmd_sat['Max']=ctr_cmd_max
    self.ctr_cmd_sat['Min']=ctr_cmd_min
    return

  def resetErrorIntegral():
    self.error_integral = 0.0
    return

  def reset():
    self.prev_time_stamp_ros = rospy.Time(0.0, 0.0)
    self.prev_error = 0.0
    self.error_integral = 0.0
    return

  def call(self, curr_time_stamp, error):

    #
    control_cmd = 0.0

    # Compute delta time
    delta_time = 0.0
    if(self.prev_time_stamp_ros == rospy.Time(0.0, 0.0)):
      delta_time = 0.0
    else:
      delta_time = (curr_time_stamp - self.prev_time_stamp_ros).to_sec()

   
    self.p = error * self.gains['P']

    # Compute integral
    # TODO by student
    # use:
    # error
    # delta_time
    # self.error_integral
    
    self.i = (self.error_integral * delta_time) * self.gains['I']

    # Anti wind-up of the error integral
    # TODO by student
    # use:
    # self.error_integral    
        
    if self.error_integral > self.anti_windup['MaxI']:
      self.error_integral = self.anti_windup['MaxI']
    elif self.error_integral < self.anti_windup['MinI']:
      self.error_integral = self.anti_windup['MinI']

    # compute error derivative
    # TODO by student
    # use
    # error_derivative
    # error, self.prev_error
    # delta_time
    error_derivative = error - self.prev_error
    self.d = 0
    if delta_time > 0:
        self.d = error_derivative / delta_time
    
    # K  
    # command
    # TODO (complete) by student
    # use
    # error_integral, error_derivative
    # self.gains
    control_cmd += self.p  + self.i + (self.d * self.gains['D'])

    # saturation of the control command
    if(control_cmd>self.ctr_cmd_sat['Max']):
      control_cmd = self.ctr_cmd_sat['Max']
    if(control_cmd<self.ctr_cmd_sat['Min']):
      control_cmd = self.ctr_cmd_sat['Min']

    # Update timestamp
    self.prev_time_stamp_ros = curr_time_stamp

    # Update prev error
    self.prev_error = error

    # Update error integral
    self.error_integral += error

    # End
    return control_cmd


