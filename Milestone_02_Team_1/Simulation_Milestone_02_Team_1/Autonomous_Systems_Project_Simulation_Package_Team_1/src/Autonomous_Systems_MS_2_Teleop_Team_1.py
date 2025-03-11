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


class Teleop:
    def __init__(self):
        rospy.init_node('Teleop')

        throttle_input = 0.0
        steering_input = 0.0
        braking_input = 0.0

        throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=10)
        steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=10)
        braking_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=10)
        
        rospy.Subscriber('/twist', TwistStamped, self.twist_callback)
        rospy.Subscriber('/steering_state', Float64, self.steering_callback)
        rospy.Subscriber('odom', Odometry, self.odom_callback)

        rate = rospy.Rate(10) 

        self.v = 0.0
        self.w = 0.0
        self.steering_angle = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.heading = 0.0


        while not rospy.is_shutdown():
            key = self.get_key()
            key_pressed = ""
            #print(f"key2:{key}")
            if key == 'w' or key == '[A':  
                throttle_input = min(throttle_input + 0.05, 1.0)
                braking_input = 0.0
                key_pressed = "UP"
            elif key == 's' or key == '[B':  
                throttle_input = max(throttle_input - 0.05, 0.0)
                key_pressed = "DOWN"
            elif key == 'd' or key == '[D': 
                steering_input = min(steering_input + 0.1, 0.8)
                key_pressed = "LEFT"
            elif key == 'a' or key == '[C': 
                steering_input = max(steering_input - 0.1, -0.8)
                key_pressed = "RIGHT"
            elif key == 'b':
                throttle_input = 0
                braking_input = 60.0   
                key_pressed = "B" 
            elif key == 'q':  # Quit
                rospy.loginfo("Shutting down Teleoperation Node.")
                break
            
            throttle_pub.publish(Float64(throttle_input))
            steering_pub.publish(Float64(steering_input))
            braking_pub.publish(Float64(braking_input))
            rospy.loginfo(f"Key Pressed: {key_pressed}")
            rospy.loginfo(f"Throttle: {throttle_input}, Steering: {steering_input}")
            rospy.loginfo(f"Linear Velocity: {self.v}")
            rospy.loginfo(f"Angular Velocity :{self.w}")
            rospy.loginfo(f"Current Steering Angle: {self.steering_angle}")
            rospy.loginfo(f"Position x: {self.position_x}")
            rospy.loginfo(f"Position y: {self.position_y}")
            rospy.loginfo(f"heading: {self.heading}")

            
            rate.sleep()
        
    def get_key(self):

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
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print(f"{key}")
        return key


    def twist_callback(self, msg):
        self.v = msg.twist.linear.x
        self.w = msg.twist.angular.z
        #rospy.loginfo(f"Velocity: {v}")
        #rospy.loginfo(f"Angular Velocity :{w}")

    def steering_callback(self, msg):
        self.steering_angle = msg.data
        #rospy.loginfo(f"Current Steering Angle: {steering_angle}")

    def odom_callback(self, msg):
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        orientation_x = msg.pose.pose.orientation.x
        orientation_y = msg.pose.pose.orientation.y
        orientation_z = msg.pose.pose.orientation.z
        orientation_w = msg.pose.pose.orientation.w

        siny_cosp = 2 * (orientation_w * orientation_z + orientation_x * orientation_y)
        cosy_cosp = 1 - 2 * (orientation_y * orientation_y + orientation_z * orientation_z)

        self.heading = math.atan2(siny_cosp, cosy_cosp)

        #rospy.loginfo(f"Position: {position_x}")
        #rospy.loginfo(f"heading: {heading}")
        


if __name__ == '__main__':
    try:
        node = Teleop()
    except rospy.ROSInterruptException:
        pass