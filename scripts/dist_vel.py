#! /usr/bin/env python

import rospy
import assignment_2_2023
from assignment_2_2023.srv import DistVel, DistVelResponse  # Update with your package and service name
from assignment_2_2023.msg import Posvel  # Update with your package and message name
from geometry_msgs.msg import Point
import math

# Class definition dist being the distance from the robot to the goal and mu
# being the average speed
class RobotDistMU:
    def __init__(self):

        # Initialize variables
        self.robot_pos_vel = None
        self.target_position = Point(0, 0, 0)  # Assuming a default target position
        self.dist = 0.0
        self.mu = 0.0
        self.vel_history_x = []
        self.vel_history_y = []
        self.time_history = []

        self.window_size = rospy.get_param('window_size', 10)

        # Set up a subscriber to receive robot's position and velocity
        rospy.Subscriber('/pos', Posvel, self.robot_pos_vel_callback)

        # Set up a service server to respond to requests for distance and mu
        rospy.Service('/get_distance_mu', DistVel, self.handle_get_distance_mu)

    def robot_pos_vel_callback(self, data):
        # Callback for receiving robot's position and velocity messages
        self.robot_pos_vel = data
        self.vel_history_x.append(data.vx)
        self.vel_history_y.append(data.vy)
        self.time_history.append(rospy.get_time())
        self.calculate_dist_vel()

    def calculate_av_speed(self):
        # Calculate average speed based on velocity history
        if len(self.vel_history_x) > 1:
            total_time = self.time_history[-1] - self.time_history[0]
            if total_time > 0:
                av_speed_x = sum(self.vel_history_x) / len(self.vel_history_x)
                av_speed_y = sum(self.vel_history_y) / len(self.vel_history_y)
                return av_speed_x, av_speed_y
        return 0, 0

    def calculate_dist_vel(self):
        # Calculate distance from robot to target based on current position
        if self.robot_pos_vel is not None:
            dx = self.target_position.x - self.robot_pos_vel.x
            dy = self.target_position.y - self.robot_pos_vel.y
            self.dist = math.sqrt(dx**2 + dy**2)

            # Calculate mu (average speed) based on velocity history
            average_speed_x, average_speed_y = self.calculate_av_speed()
            self.mu = (average_speed_x + average_speed_y) / 2.0

    def handle_get_distance_mu(self, req):
        # Service server callback to respond with distance and mu
        response = DistVelResponse()
        response.dist = self.dist
        response.mu = self.mu
        return response

def main():
    rospy.init_node('robot_dist_mu')
    service_node = RobotDistMU()
    rospy.spin()

if __name__ == '__main__':
    main()

