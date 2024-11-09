#!/usr/bin/env python3
from geometry_msgs.msg import Twist
from math import pi, atan2, sqrt
from nav_msgs.msg import Path, Odometry
import rospy

class NavigationActualizer:
    def __init__(self):
        self.max_linear_speed = 1.5
        self.max_angular_speed = 1.5
        self.waypoints = []
        self.current_waypoint_index = 1
        self.waypoint_threshold = 0.2   # Distance threshold to consider waypoint reached
        self.max_deviation = 1          # Maximum allowed deviation from planned path
        
        self.look_ahead_distance = 0.8

        # Subscribers and Publishers
        rospy.Subscriber('/path_planning/path', Path, self.path_callback)
        rospy.Subscriber('/diffbot/mobile_base_controller/odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/diffbot/mobile_base_controller/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(30)
        
        self.current_pose = None
        self.initial_pose = None

    def run(self):
        while not rospy.is_shutdown():
            if self.waypoints and self.current_pose and self.current_waypoint_index < len(self.waypoints):
                # Check if current waypoint is reached
                if self.is_waypoint_reached():
                    self.current_waypoint_index += 1
                    if self.current_waypoint_index >= len(self.waypoints):
                        rospy.loginfo("Final waypoint reached!")
                        self.stop_robot()
                        continue
                else:
                    linear_speed, angular_speed = self.compute_speed()
                    self.publish_cmd_vel(linear_speed, angular_speed)

                # Check for path deviation
                if self.check_path_deviation():
                    rospy.logwarn("Significant path deviation detected!")
                    # TODO: Implement path replanning
                    # self.request_path_replanning()
            self.rate.sleep()

    def compute_speed(self):
        """
        Computes linear and angular speeds based on the current pose and current waypoint.
        """
        x_0, y_0, yaw_0 = self.get_current_pose()
        x_t, y_t = self.waypoints[self.current_waypoint_index]
        
        dx = x_t - x_0
        dy = y_t - y_0
        distance = sqrt(dx*dx + dy*dy)
        
        yaw_t = atan2(dy, dx)
        yaw_diff = self.normalize_angle(yaw_t - yaw_0)
        
        if abs(yaw_diff) > pi/6:  # 30 degrees
            linear_speed = 0.0
            angular_speed = self.max_angular_speed * yaw_diff / pi
        else:
            linear_speed = self.max_linear_speed * (1 - abs(yaw_diff)/(pi/3))
            linear_speed = max(0.1, linear_speed) if distance > self.waypoint_threshold else linear_speed
            
            angular_speed = self.max_angular_speed * yaw_diff / (pi/2)
        
        linear_speed = max(0.0, min(linear_speed, self.max_linear_speed))
        angular_speed = max(-self.max_angular_speed, min(angular_speed, self.max_angular_speed))
        
        rospy.logdebug(f"Distance: {distance:.2f}, Yaw diff: {yaw_diff:.2f}")
        rospy.logdebug(f"Target: ({x_t:.2f}, {y_t:.2f}), Current: ({x_0:.2f}, {y_0:.2f})")
        rospy.logdebug(f"Speeds -> Linear: {linear_speed:.2f}, Angular: {angular_speed:.2f}")
        
        return linear_speed, angular_speed

    def is_waypoint_reached(self):
        """
        Checks if the current waypoint has been reached.
        """
        """
        Checks if the current waypoint has been reached with improved criteria
        """
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            return False
            
        x_0, y_0, yaw_0 = self.get_current_pose()
        x_t, y_t = self.waypoints[self.current_waypoint_index]
        
        distance = sqrt((x_t - x_0)**2 + (y_t - y_0)**2)

        yaw_t = atan2(y_t - y_0, x_t - x_0)
        yaw_diff = abs(self.normalize_angle(yaw_t - yaw_0))
        
        return distance < self.waypoint_threshold and yaw_diff < pi/4

    def check_path_deviation(self):
        """
        Checks if the robot has deviated too far from the planned path.
        """
        if len(self.waypoints) <= self.current_waypoint_index + 1:
            return False
            
        x_0, y_0, _ = self.get_current_pose()
        x_1, y_1 = self.waypoints[self.current_waypoint_index]
        x_2, y_2 = self.waypoints[self.current_waypoint_index + 1]
        
        # Calculate distance from point to line segment
        numerator = abs((x_2 - x_1)*(y_1 - y_0) - (x_1 - x_0)*(y_2 - y_1))
        denominator = sqrt((x_2 - x_1)**2 + (y_2 - y_1)**2)
        
        if denominator == 0:
            return False
            
        deviation = numerator / denominator
        return deviation > self.max_deviation

    def stop_robot(self):
        """
        Stops the robot by publishing zero velocities.
        """
        self.publish_cmd_vel(0.0, 0.0)

    def path_callback(self, msg):
        """
        Callback for the planned path, updating waypoints.
        """
        if len(self.waypoints) == 0:
            self.waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
            self.current_waypoint_index = 1
            rospy.loginfo(f"New waypoints received: {self.waypoints}")

    def odom_callback(self, msg):
        """
        Callback for odometry data, updating the current pose.
        """
        position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        self.current_pose = (
            position.x,
            position.y,
            self.quaternion_to_yaw(orientation_q)
        )

        if self.initial_pose is None:
            self.initial_pose = (
                position.x,
                position.y,
                self.quaternion_to_yaw(orientation_q)
            )

    def publish_cmd_vel(self, linear_speed, angular_speed):
        """
        Publishes the computed velocities as a Twist message.
        """
        twist_msg = Twist()
        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = angular_speed
        self.cmd_vel_pub.publish(twist_msg)

    
    def get_current_pose(self):
        x_0, y_0, yaw_0 = self.current_pose
        x_init, y_init, yaw_init = self.initial_pose
        x_0 = x_0 - x_init
        y_0 = y_0 - y_init
        yaw_0 = yaw_0 - yaw_init
        return (x_0, y_0, yaw_0)
    
    @staticmethod
    def quaternion_to_yaw(quaternion):
        """
        Converts a quaternion to a yaw angle.
        """
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        """
        Normalizes an angle to the range [-pi, pi].
        """
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

# TODO: Path replanning implementation
"""
def request_path_replanning(self):
    # This method would be implemented to request a new path when significant deviation is detected
    # 1. Stop the robot
    self.stop_robot()
    
    # 2. Request new path from path planner
    # Example implementation:
    # current_x, current_y, _ = self.current_pose
    # goal_x, goal_y = self.waypoints[-1]
    # path_request = PathPlanningRequest()
    # path_request.start = Point(current_x, current_y)
    # path_request.goal = Point(goal_x, goal_y)
    # new_path = path_planner_service(path_request)
    
    # 3. Update waypoints with new path
    # self.waypoints = [(point.x, point.y) for point in new_path.points]
    # self.current_waypoint_index = 0
    pass
"""

if __name__ == "__main__":
    rospy.init_node('navigation_actualizer')
    actualizer = NavigationActualizer()
    actualizer.run()
    rospy.spin()