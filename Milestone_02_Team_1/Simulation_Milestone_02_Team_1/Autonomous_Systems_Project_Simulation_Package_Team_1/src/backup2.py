#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import sys, select, termios, tty
import math

# Define the keys for throttle, brake, and steering
key_bindings = {
    'w': (0.1, 0.0, 0.0),  # Increase throttle
    's': (-0.1, 0.0, 0.0), # Decrease throttle
    'a': (0.0, 0.0, 0.1),  # Steer left
    'd': (0.0, 0.0, -0.1), # Steer right
    'x': (0.0, 10.0, 0.0), # Apply brake
}

# Define steering limits in degrees and convert to radians for internal use
STEERING_LIMIT_DEG = 35.0  # Steering limit in degrees
STEERING_LIMIT_RAD = math.radians(STEERING_LIMIT_DEG)  # Convert to radians

class TeleopNode:
    def __init__(self):
        rospy.init_node('audibot_teleop', anonymous=True)

        # Publishers for steering, brake, and throttle
        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=10)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=10)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=10)

        # Subscribers for vehicle states
        self.steering_sub = rospy.Subscriber('/steering_cmd', Float64, self.steering_callback)
        self.twist_sub = rospy.Subscriber('/twist', TwistStamped, self.twist_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Initialize control values
        self.throttle = 0.0  # Throttle percentage (0 to 1)
        self.brake = 0.0     # Brake torque (Nm)
        self.steering = 0.0  # Steering angle (radians)

        # Vehicle state variables
        self.current_steering = 0.0
        self.velocity = 0.0
        self.position = None
        self.orientation = None

        # Terminal settings for reading keyboard input
        self.settings = termios.tcgetattr(sys.stdin)

    def steering_callback(self, msg):
        # Current steering angle (in radians)
        self.current_steering = msg.data

    def twist_callback(self, msg):
        # Current velocity (linear.x is the speed)
        self.velocity = msg.twist.linear.x

    def odom_callback(self, msg):
        # Current position and orientation
        self.position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.orientation) = euler_from_quaternion(orientation_list)  # Only yaw is needed

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def print_vehicle_states(self):
        # Print all vehicle states to the terminal
        rospy.loginfo("\n--- Vehicle States ---")
        rospy.loginfo(f"Steering Angle: {math.degrees(self.current_steering):.2f} deg")  # Convert to degrees
        rospy.loginfo(f"Velocity: {self.velocity:.2f} m/s")
        if self.position:
            rospy.loginfo(f"Position: [x: {self.position.x:.2f}, y: {self.position.y:.2f}, z: {self.position.z:.2f}]")
        if self.orientation:
            rospy.loginfo(f"Orientation (Yaw): {math.degrees(self.orientation):.2f} deg")  # Convert to degrees

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz control loop
        while not rospy.is_shutdown():
            key = self.get_key()
            if key in key_bindings:
                throttle_delta, brake_delta, steering_delta = key_bindings[key]

                # Debug: Print the key pressed and the corresponding steering delta
                rospy.loginfo(f"Key Pressed: {key}, Steering Delta: {steering_delta}")

                # Update throttle and brake
                self.throttle = max(0.0, min(1.0, self.throttle + throttle_delta))  # Clamp throttle between 0 and 1
                self.brake = max(0.0, self.brake + brake_delta)                    # Brake cannot be negative

                # Update steering with limits
                new_steering = self.steering + steering_delta
                if new_steering > STEERING_LIMIT_RAD:
                    new_steering = STEERING_LIMIT_RAD  # Clamp to the right limit
                elif new_steering < -STEERING_LIMIT_RAD:
                    new_steering = -STEERING_LIMIT_RAD  # Clamp to the left limit
                self.steering = new_steering

                # Debug: Print the updated steering value
                rospy.loginfo(f"Updated Steering: {math.degrees(self.steering):.2f} deg")

                # Publish control values
                self.throttle_pub.publish(self.throttle)
                self.brake_pub.publish(self.brake)
                self.steering_pub.publish(self.steering)

                # Print current control values and vehicle states
                rospy.loginfo(f"\n--- Control Commands ---")
                rospy.loginfo(f"Throttle: {self.throttle:.2f}, Brake: {self.brake:.2f} Nm, Steering: {math.degrees(self.steering):.2f} deg")  # Convert to degrees
                self.print_vehicle_states()

            elif key == '\x03':  # Ctrl+C to exit
                break

            rate.sleep()

if __name__ == '__main__':
    try:
        node = TeleopNode()
        node.run()
    except rospy.ROSInterruptException:
        pass