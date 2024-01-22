#! /usr/bin/env python

# import everything necessary
import rospy
import actionlib
import actionlib.msg
import assignment_2_2023
import threading
from std_msgs.msg import String
from assignment_2_2023.msg import Posvel
from assignment_2_2023.msg import PlanningAction, PlanningGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

# action client class definition
class ActionClient:
    def __init__(self):

        # Initialization of tha ction client for the 'reaching_goal' action server
        self.client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
        self.client.wait_for_server()

        # Publishers for the position and goal/target information
        self.pos_vel_pub = rospy.Publisher('/pos', assignment_2_2023.msg.Posvel, queue_size = 10)
        self.current_goal_pub = rospy.Publisher('/goal', Point, queue_size = 10)

        # Subscribe to the 'odo'm to receive odometry info
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, data):
        # Extract position and velocity from odometry
        pos_x = data.pose.pose.position.x 
        pos_y = data.pose.pose.position.y 
        vx = data.twist.twist.linear.x 
        vy = data.twist.twist.linear.y 

        # Create and publish Posvel message 
        pos_vel_msg = Posvel()
        pos_vel_msg.x = pos_x
        pos_vel_msg.y = pos_y
        pos_vel_msg.vx = vx
        pos_vel_msg.vy = vy
        self.pos_vel_pub.publish(pos_vel_msg)

    def send_goal(self, x, y):
        #  Create a goal with target coordinates and send to the action server
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        self.client.send_goal(goal)

        #  Publish current goal/target coordinates
        self.current_goal_pub.publish(Point(x, y, 0))
        rospy.loginfo("Goal sent: x = %f, y = %f" % (x, y))

    def cancel_goal(self):
        # Cancel all goals
        self.client.cancel_all_goals()
        rospy.loginfo("Goals cancelled")

    def listen_goals(self):
        # Continuously prompt user for input
        while not rospy.is_shutdown():
            user_input = input("Enter a goal as 'x,y', or type 's' to stop:")
            print(user_input)

            if user_input == 's':
                self.cancel_goal()
            else:
                try:
                    x, y = map(float, user_input.split(','))
                    self.send_goal(x, y)
                except ValueError:
                    rospy.loginfo("Invalid input. Please enter the goal as 'x,y' as numbers or type 'stop' to cancel") 

    

def main():
    rospy.init_node('action_client', anonymous = True)
    client = ActionClient()

    # Start a separate thread for listening to user input and sending goals
    goal_thread = threading.Thread(target=client.listen_goals)
    goal_thread.start()

    rospy.spin()
    # Wait for the goal thread to finish before exiting
    goal_thread.join()

if __name__ == '__main__':
    main()
