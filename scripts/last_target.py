#! /usr/bin/env python

import assignment_2_2023
import rospy
from assignment_2_2023.srv import LastTarget, LastTargetResponse
from geometry_msgs.msg import Point

class LastTargetService:
    def __init__(self):
        self.last_target_x = 0.0
        self.last_target_y = 0.0

        rospy.Service('/get_last_target', LastTarget, self.handle_last_target_request)
        rospy.Subscriber('/target', Point, self.handle_last_target_request)

    def handle_last_target_request(self, req):
        response = LastTargetResponse()
        if self.last_target_x is not None and self.last_target_y is not None:
            response.last_target_x = self.last_target_x
            response.last_target_y = self.last_target_y
        else:
            rospy.logwarn("No target")
            response.x = 0.0
            response.y = 0.0
        return response
        

    def last_target_callback(self, data):
        self.last_target_x = data.x
        self.last_target_y = data.y

def main():
    rospy.init_node('last_target_service')
    service = LastTargetService()
    rospy.spin()

if __name__ == '__main__':
    main()