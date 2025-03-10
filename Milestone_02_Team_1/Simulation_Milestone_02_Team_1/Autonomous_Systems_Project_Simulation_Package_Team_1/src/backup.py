#!/usr/bin/env python3

#/throttle_cmd
#/steering_cmd
#/gear_cmd


import rospy
import math
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

import tty
import sys
import termios
from select import select

def main():
    rospy.init_node('Teleop')

    throttle_input = 0.0
    steering_input = 0.0
    braking_input = 0.0

    throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=10)
    steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=10)
    braking_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=10)
    
    rospy.Subscriber('/twist', TwistStamped, twist_callback)
    rospy.Subscriber('/steering_state', Float64, steering_callback)
    rospy.Subscriber('odom', Odometry, odom_callback)

    rate = rospy.Rate(10) 


    while not rospy.is_shutdown():
        key = get_key()
        if key == 'w' or key == '[A':  
            throttle_input = min(throttle_input + 0.05, 1.0)
            braking_input = 0.0
        elif key == 's' or key == '[B':  
            throttle_input = max(throttle_input - 0.05, 0.0)
        elif key == 'd' or key == '[D': 
            steering_input = max(steering_input - 0.1, -0.8)
        elif key == 'a' or key == '[C': 
            steering_input = min(steering_input + 0.1, 0.8)
        elif key == 'b':
            throttle_input = 0
            braking_input = 50.0    
        elif key == 'q':  # Quit
            rospy.loginfo("Shutting down Teleoperation Node.")
            break
        
        throttle_pub.publish(Float64(throttle_input))
        steering_pub.publish(Float64(steering_input))
        braking_pub.publish(Float64(braking_input))
        #rospy.loginfo(f"Throttle: {throttle_input}, Steering: {steering_input}")
        
        rate.sleep()


def get_key():
    try:
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        key = ''
        rlist, _, _ = select([sys.stdin], [], [], 0.5)
        #rlist = False
        if rlist:
            key = sys.stdin.read(1)
            #print(f"{key}")
            if key == "\x1b":
                #print(f"Escape")
                key = sys.stdin.read(2)
                #print(f"key:{key}")
        else:
            key = ''
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def twist_callback(msg):
    v = msg.twist.linear.x
    w = msg.twist.angular.z
    #rospy.loginfo(f"Velocity: {v}")
    #rospy.loginfo(f"Angular Velocity :{w}")

def steering_callback(msg):
    steering_angle = msg.data
    rospy.loginfo(f"Current Steering Angle: {steering_angle}")

def odom_callback(msg):
    position_x = msg.pose.pose.position.x
    orientation_x = msg.pose.pose.orientation.x
    orientation_y = msg.pose.pose.orientation.y
    orientation_z = msg.pose.pose.orientation.z
    orientation_w = msg.pose.pose.orientation.w

    siny_cosp = 2 * (orientation_w * orientation_z + orientation_x * orientation_y)
    cosy_cosp = 1 - 2 * (orientation_y * orientation_y + orientation_z * orientation_z)

    heading = math.atan2(siny_cosp, cosy_cosp)

    #rospy.loginfo(f"Position: {position_x}")
    #rospy.loginfo(f"heading: {heading}")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass