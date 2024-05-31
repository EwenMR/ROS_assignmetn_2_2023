#! /usr/bin/env python

import assignment_2_2023
import rospy
from assignment_2_2023.srv import LastTarget, LastTargetResponse
from geometry_msgs.msg import Point

#  Class definition
class LastTargetService:
    def __init__(self):
        # Initialization of the variables used to store last target coordinates
        self.last_target_x = 0.0
        self.last_target_y = 0.0

        rospy.Subscriber('/goal', Point, self.last_target_callback)
        rospy.Service('/get_last_target', LastTarget, self.handle_last_target_request)
        

    def handle_last_target_request(self, req):
        # Service server callback to respond with last target coordinates
        response = LastTargetResponse()
        response.last_target_x = self.last_target_x
        response.last_target_y = self.last_target_y
        return response
        

    def last_target_callback(self, data):
        # Callback for receiving current goal/target coordinates
        self.last_target_x = data.x
        self.last_target_y = data.y

def main():
    rospy.init_node('last_target_service')
    service = LastTargetService()
    rospy.spin()

if __name__ == '__main__':
    main()