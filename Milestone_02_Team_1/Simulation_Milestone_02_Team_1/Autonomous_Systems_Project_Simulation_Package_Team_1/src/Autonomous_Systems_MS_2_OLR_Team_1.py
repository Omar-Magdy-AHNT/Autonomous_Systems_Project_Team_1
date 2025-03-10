#!/usr/bin/env python3

#/throttle_cmd
#/steering_cmd
#/gear_cmd


import rospy
import math

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

def main():
    rospy.init_node('openloop')

    throttle_input = rospy.get_param('throttle', 0.3)
    steering_input = rospy.get_param('steering', 0.2)

    throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=10)
    steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=10)
    
    rospy.Subscriber('/twist', TwistStamped, twist_callback)
    rospy.Subscriber('/steering_state', Float64, steering_callback)
    rospy.Subscriber('odom', Odometry, odom_callback)

    rate = rospy.Rate(10) #10 Hz

    while not rospy.is_shutdown():
        throttle_pub.publish(Float64(throttle_input))
        steering_pub.publish(Float64(steering_input))

        rate.sleep()

def twist_callback(msg):
    v = msg.twist.linear.x
    w = msg.twist.angular.z
    rospy.loginfo(f"Velocity: {v}")
    rospy.loginfo(f"Angular Velocity :{w}")

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

    rospy.loginfo(f"Position: {position_x}")
    rospy.loginfo(f"heading: {heading}")




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass