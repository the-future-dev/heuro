#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_straight():
    # Initialize the ROS node
    rospy.init_node('move_straight', anonymous=True)
    
    # Create a publisher to the 'cmd_vel' topic
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    # Set the rate at which to publish messages (10 Hz)
    rate = rospy.Rate(10)
    
    # Create a Twist message to specify linear and angular velocities
    velocity_msg = Twist()
    
    # Set the desired linear velocity (x-direction) and keep angular velocity zero
    velocity_msg.linear.x = 1.0  # Adjust the speed as needed
    velocity_msg.linear.y = 0.0
    velocity_msg.linear.z = 0.0
    velocity_msg.angular.x = 0.0
    velocity_msg.angular.y = 0.0
    velocity_msg.angular.z = 0.0

    # Continue publishing the velocity message until the node is stopped
    while not rospy.is_shutdown():
        velocity_publisher.publish(velocity_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_straight()
    except rospy.ROSInterruptException:
        pass
