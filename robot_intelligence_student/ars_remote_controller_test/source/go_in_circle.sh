#!/bin/bash

echo "Go in infinite cercles":

rostopic pub -1 /robot_cmd_stamped geometry_msgs/TwistStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
twist:
  linear:
    x: 2.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 1.8" 