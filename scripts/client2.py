#!/usr/bin/env python

import rospy
import actionlib
from assignment_2_2023.msg import Posvel, PlanningAction, PlanningGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import sys
import select

class ActionClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/otw_goal', PlanningAction)
        self.client.wait_for_server()

        self.pos_vel_pub = rospy.Publisher('/pos', Posvel, queue_size=10)
        self.current_goal_pub = rospy.Publisher('/goal', Point, queue_size=10)

        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.prompt_user = True

        # Timer for non-blocking input check
        rospy.Timer(rospy.Duration(1), self.check_input)

    def check_input(self, event):
        if self.prompt_user:
            print("Enter a goal as 'x,y', or type 's' to stop:")
            self.prompt_user = False

        if select.select([sys.stdin], [], [], 0)[0]:
            user_input = sys.stdin.readline().strip()
            self.prompt_user = True  # Reset the prompt for next input
            if user_input == 's':
                self.cancel_goal()
            else:
                try:
                    x, y = map(float, user_input.split(','))
                    self.send_goal(x, y)
                except ValueError:
                    rospy.loginfo("Invalid input. Please enter the goal as 'x,y' as numbers or type 's' to cancel")

    def odom_callback(self, data):
        pos_x = data.pose.pose.position.x
        pos_y = data.pose.pose.position.y

        vx = data.twist.twist.linear.x
        vy = data.twist.twist.linear.y

        pos_vel_msg = Posvel()
        pos_vel_msg.x = pos_x
        pos_vel_msg.y = pos_y
        pos_vel_msg.vx = vx
        pos_vel_msg.vy = vy

        self.pos_vel_pub.publish(pos_vel_msg)

    def send_goal(self, x, y):
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        self.client.send_goal(goal, feedback_cb=self.feedback_callback, done_cb=self.done_callback)

        self.current_goal_pub.publish(Point(x, y, 0))
        rospy.loginfo("Goal sent: x = %f, y = %f" % (x, y))

    def cancel_goal(self):
        self.client.cancel_all_goals()
        rospy.loginfo("Goals cancelled")

    def done_callback(self, state, result):
        rospy.loginfo("Action at state: %s and completed: %s" % (str(state), str(result.success)))

    def feedback_callback(self, feedback):
        rospy.loginfo("Feedback: current position x:%f, y:%f" % (feedback.current_x, feedback.current_y))

def main():
    rospy.init_node('action_client')
    ActionClient()
    rospy.spin()

if __name__ == '__main__':
    main()
