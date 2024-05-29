"""
\file RobotDistMU.py
\brief This module defines the class responsible for calculating the distance to the target as well as the average speed.

The class sets up a ROS subscriber to update target coordinates and provides these coordinates through a ROS service.

\author
Ewen Gay-Semenkoff


\subscribers
- /pos

\services
- /det_distance_mu    
"""

import rospy
import assignment_2_2023
from assignment_2_2023.srv import DistVel, DistVelResponse
from assignment_2_2023.msg import Posvel
from geometry_msgs.msg import Point
import math

"""
\class RobotDistMU
\brief A class to manage the robot position and velocity data to compute the distance to the target and the average speed of the robot.

\attributes
- robot_pos_vel (Posvel): stores the latest position and velocity of the robot.
- target_position (Point): the goal position the robot is currently moving towards.
- dist (float): the calculated distance from the robot to the target.
- mu (float): the calculated average speed of the robot towards the target.
- vel_history_x (list): list of x axis velocity measurements.
- vel_history_y (list): list of y axis velocity measurements.
- time_history (list): list of the timestamps corresponding to the velocity measurements.

\methods
- __init__: Initializes the RobotDistMU object, setting up the subscriber and service.
- robot_pos_vel_callback: Uses a callback to update robot's position and velocity.
- calculate_av_speed: Calculates average speed based on the velocity history.
- calculate_dist_vel: Calculates distance to target and updates the average speed (mu).
- handle_get_distance_mu: Provides the distance and average speed (mu) as a service response.
"""
class RobotDistMU:
    """
    \brief Initializes the RobotDistMU object, setting up the subscriber and service.
    """
    def __init__(self):
    """
    \brief Callback function updates the position and velocity from /pos topic and recalculates distance and velocity.
    
    \param data (Posvel): The position and velocity message of the robot.
    """
        self.robot_pos_vel = None
        self.target_position = Point(0, 0, 0)  # Assuming a default target position
        self.dist = 0.0
        self.mu = 0.0
        self.vel_history_x = []
        self.vel_history_y = []
        self.time_history = []
        
        self.window_size = rospy.get_param('window_size', 10)

        rospy.Subscriber('/pos', Posvel, self.robot_pos_vel_callback)
        rospy.Service('/get_distance_mu', DistVel, self.handle_get_distance_mu)

    
    def robot_pos_vel_callback(self, data):
    """
    \brief Computes the average speed of the robot based on the velocity history.
    
    \return tuple: Average speed in x and y directions
    """
        self.robot_pos_vel = data
        self.vel_history_x.append(data.vx)
        self.vel_history_y.append(data.vy)
        self.time_history.append(rospy.get_time())
        self.calculate_dist_vel()

    
    def calculate_av_speed(self):
    """
    \brief Calculates the distance from the robot to the target position and updates the variable 'mu'.
    """
        if len(self.vel_history_x) > 1:
            total_time = self.time_history[-1] - self.time_history[0]
            if total_time > 0:
                av_speed_x = sum(self.vel_history_x) / len(self.vel_history_x)
                av_speed_y = sum(self.vel_history_y) / len(self.vel_history_y)
                return av_speed_x, av_speed_y
        return 0, 0

    
    def calculate_dist_vel(self):
    """
    \brief Responds to the service requests with the current distance and average speed (mu).
    
    \param req: the service request (empty for DistVel service).
    
    \return DistVelResponse: the service response containing distance and mu.
    """
        if self.robot_pos_vel is not None:
            dx = self.target_position.x - self.robot_pos_vel.x
            dy = self.target_position.y - self.robot_pos_vel.y
            self.dist = math.sqrt(dx**2 + dy**2)
            average_speed_x, average_speed_y = self.calculate_av_speed()
            self.mu = (average_speed_x + average_speed_y) / 2.0

    
    def handle_get_distance_mu(self, req):
    """
    \brief Initializes the ROS node and the RobotDistMU service.
    """
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

