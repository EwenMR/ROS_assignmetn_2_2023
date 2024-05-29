"""
\file LastTargetService.py
\brief This module defines the LastTargetService class responsible for managing and providing the last known target coordinates via a ROS service.

\author
Ewen Gay-Semenkoff

The class sets up a ROS subscriber to update target coordinates and provides these coordinates through a ROS service.

\subscribers
- /pos

\services
- /get_distance_mu
"""

import assignment_2_2023
import rospy
from assignment_2_2023.srv import LastTarget, LastTargetResponse
from geometry_msgs.msg import Point

"""
\class LastTargetService
\brief Class to manage and provide the last known target coordinates via ROS service.

The class includes methods to update the last known target coordinates upon receiving a new target and to provide these coordinates as a service response.
"""
class LastTargetService:
    """
    \brief Initializes the LastTargetService object, setting up the subscriber and service.
    
    \attributes
    - last_target_x (float): The x-coordinate of the last known target.
    - last_target_y (float): The y-coordinate of the last known target.

    \methods
    - __init__: Initializes the LastTargetService object, setting up the subscriber and service.
    - handle_last_target_request: Responds to service requests with the last known target coordinates.
    - last_target_callback: Callback for receiving current goal coordinates and updating the internal state.

    """
    def __init__(self):
    """
    \brief Responds to service requests with the last known target coordinates.
    
    \param req The service request (empty for LastTarget service).
    \return LastTargetResponse The service response containing the last target coordinates.
    """
        self.last_target_x = 0.0
        self.last_target_y = 0.0

        rospy.Subscriber('/goal', Point, self.last_target_callback)
        rospy.Service('/get_last_target', LastTarget, self.handle_last_target_request)
    
    
    def handle_last_target_request(self, req):
     """
    \brief Callback for receiving current goal coordinates and updating the internal state.
    
    \param data The goal position message containing the coordinates (type: Point).
    """
        response = LastTargetResponse()
        response.last_target_x = self.last_target_x
        response.last_target_y = self.last_target_y
        return response

   
    def last_target_callback(self, data):
    """
    \brief Initializes the ROS node and the LastTargetService.
    """
        self.last_target_x = data.x
        self.last_target_y = data.y


def main():
    rospy.init_node('last_target_service')
    service = LastTargetService()
    rospy.spin()

if __name__ == '__main__':
    main()

