#!/usr/bin/env python

import rospy
import math
import time
import sys 


from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


x = 0
y = 0
z = 0
yaw = 0



def Odometry_Callback(msg):
    global x
    global y
    global z

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    yaw = msg.pose.pose.orientation.z

    rospy.loginfo('value of x is: %f', x)
    rospy.loginfo('value of y is: %f', y)
    rospy.loginfo('value of z is: %f', z)
    
def takeoff():
    pub = rospy.Publisher('/takeoff', Empty, queue_size=1, latch=True)
    pub.publish(Empty())
    time.sleep(2.)

def land():
    print('land')
    pub = rospy.Publisher('/land', Empty, queue_size=1, latch=True)
    pub.publish(Empty())
    time.sleep(4.)




def moveX(speed, distance, is_forward):
        
    velocity_publisher = rospy.Publisher("robot_cmd_stamped", TwistStamped, queue_size=1)
    velocity_message = TwistStamped()

    global x, y
    x0=x
    y0=y

    if (is_forward):
        velocity_message.twist.linear.x =abs(speed)
    else:
        velocity_message.twist.linear.x =-abs(speed)

    distance_moved = 0.0
    #loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    

    while True :
            rospy.loginfo("Nao_Drone moves forwards")
            velocity_publisher.publish(velocity_message)
            loop_rate.sleep()    
            distance_moved = abs(math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
            print  distance_moved               
            if (distance_moved >= distance):
                rospy.loginfo("reached")
                break
        
    velocity_message.twist.linear.x =0
    velocity_publisher.publish(velocity_message)
    
def moveY(speed, distance, is_forward):
        
    velocity_publisher = rospy.Publisher("robot_cmd_stamped", TwistStamped, queue_size=1)
    velocity_message = TwistStamped()

    global x, y
    x0=x
    y0=y

    if (is_forward):
        velocity_message.twist.linear.y =abs(speed)
    else:
        velocity_message.twist.linear.y =-abs(speed)

    distance_moved = 0.0
    #loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    

    while True :
            rospy.loginfo("Nao_Drone moves forwards")
            velocity_publisher.publish(velocity_message)
            loop_rate.sleep()    
            distance_moved = abs(math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
            print  distance_moved               
            if (distance_moved >= distance):
               rospy.loginfo("reached")
               break
        
    velocity_message.linear.y =0
    velocity_publisher.publish(velocity_message)

def moveZ(speed, distance, is_forward):
        
    velocity_publisher = rospy.Publisher("robot_cmd_stamped", TwistStamped, queue_size=1)
    velocity_message = TwistStamped()

    global z
    z0=z

    if (is_forward):
        velocity_message.twist.linear.z =abs(speed)
    else:
        velocity_message.twist.linear.z =-abs(speed)

    distance_moved = 0.0
    #loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    

    while True :
            rospy.loginfo("Drone goes UP")
            velocity_publisher.publish(velocity_message)
            loop_rate.sleep()    
            distance_moved = abs(math.sqrt((z-z0) ** 2))
            print  distance_moved               
            if (distance_moved >= distance):
                rospy.loginfo("reached")
                break
        
    velocity_message.linear.z =0
    velocity_publisher.publish(velocity_message)

def rotate (angular_speed_degree, relative_angle_degree, clockwise):
    
    velocity_publisher = rospy.Publisher("robot_cmd_stamped", TwistStamped, queue_size=0)
    velocity_message = TwistStamped()
    angular_speed=math.radians(abs(angular_speed_degree))

    if (clockwise):
        velocity_message.twist.angular.z =-abs(angular_speed)
    else:
        velocity_message.twist.angular.z =abs(angular_speed)

    loop_rate = rospy.Rate(5) 
    t0 = rospy.Time.now().to_sec()

    while True :
        rospy.loginfo("Nao_Drone rotates")
        velocity_publisher.publish(velocity_message)

        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_degree
        loop_rate.sleep()

        if  (current_angle_degree>relative_angle_degree):
            rospy.loginfo("reached")
            break

    velocity_message.twist.angular.z =0
    velocity_publisher.publish(velocity_message)

def home():

    global x, y, yaw

    desired_angle = 0.0
    delta_y = 0-y
    delta_x = 0-x
    desired_distance = abs(math.sqrt((x ** 2) + (y ** 2))) 

    if (x < 0 and y < 0):
        desired_angle = math.asin(delta_y/desired_distance)
        desired_angle = math.degrees(desired_angle)

    elif (x < 0 and y > 0):
        desired_angle = math.asin(delta_y/desired_distance)
        desired_angle = math.degrees(desired_angle)

    elif (x > 0 and y < 0):
        angle = math.asin(delta_y/desired_distance)
        angle = math.degrees(desired_angle)
        desired_angle = 180 - angle

    elif (x > 0 and y > 0):
        angle = math.asin(delta_y/desired_distance)
        angle = math.degrees(desired_angle)
        desired_angle = 180 + angle
    elif (x == 0 and y>0):
        desired_angle = 270
    elif (x == 0 and y < 0):
        desired_angle = 90
    elif (y == 0 and x > 0):
        desired_angle = 180
    elif (y == 0 and x < 0):
        desired_angle = 0

    else: 
        print("you are already at base")

    if (x !=0 and y != 0):
        relative_angle = desired_angle - yaw

        rotate(10, relative_angle, True)
        time.sleep(2)
        moveX(0.1, desired_distance, True)
        land()

def _killMoveCallback(self, msg):
        """
        INTERNAL METHOD, callback triggered when a message is received on the
        '/move_base_simple/cancel' topic. This callback is used to stop the
        robot's base from moving

        Parameters:
            msg - an empty ROS message, with the Empty type
        """
        self.robot.moveTo(0, 0, 0, _async=True) 
   