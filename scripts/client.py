"""
\package ActionClient
\brief This module defines the ActionClient class responsible for interfacing with a ROS action server to send goals, publish position and velocity updates, and manage user inputs for dynamic interaction with a simulated robot environment.

\author
Ewen Gay-Semenkoff ewengay2002@hotmail.com

\subscribers
- /odom
- /reaching_goal/feedback

\publishers
- /pos
- /goal

\services
- /gazebo/reset_world

\actions
- /reaching_goal
"""

import rospy
import actionlib
import threading
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_srvs.srv import Empty
from assignment_2_2023.msg import Posvel, PlanningAction, PlanningGoal
from assignment_2_2023.msg import PlanningFeedback  # Assuming this is correct, was not imported in the initial script

## Action client class definition
class ActionClient:
    """
    \class ActionClient
    \brief A class to interact with a ROS action server, publish and subscribe to ROS topics, and handle dynamic user inputs.

    \attributes
    - client (actionlib.SimpleActionClient): The action client to send goals to the 'reaching_goal' action server.
    - pos_vel_pub (rospy.Publisher): Publisher for robot's position and velocity.
    - current_goal_pub (rospy.Publisher): Publisher for the current goal/target position of the robot.

    \methods
    - feedback_callback: Logs feedback from the action server.
    - odom_callback: Processes odometry data to update and publish the robot's current position and velocity.
    - send_goal: Sends a goal to the action server.
    - cancel_goal: Cancels all pending goals at the action server.
    - reset_world: Calls a service to reset the simulation environment.
    - obst_dist: Subscribes to a topic to get distance measurements (implementation needed).
    - listen_goals: Listens to user input to dynamically manage goals and actions.
    """

    def __init__(self):
        """
        \brief Initializes the ActionClient object, setting up the action client, publishers, and subscribers.
        """
        self.client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        self.client.wait_for_server()

        self.pos_vel_pub = rospy.Publisher('/pos', Posvel, queue_size=10)
        self.current_goal_pub = rospy.Publisher('/goal', Point, queue_size=10)

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/reaching_goal/feedback', String, self.feedback_callback)  # Assuming feedback is of type String

    def feedback_callback(self, feedback):
        """
        \brief Processes feedback from the action server.
        
        \param feedback (String): The feedback message from the action server.
        """
        rospy.loginfo("Feedback is: %s", feedback.data)

    def odom_callback(self, data):
        """
        \brief Callback function to process odometry data, update the robot's position and velocity, and publish these updates.

        \param data (Odometry): The odometry data containing position and velocity information.
        """
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
        """
        \brief Creates and sends a goal to the action server and publishes the goal coordinates.

        \param x (float): The x-coordinate of the target goal.
        \param y (float): The y-coordinate of the target goal.
        """
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        self.client.send_goal(goal)
        self.current_goal_pub.publish(Point(x, y, 0))
        rospy.loginfo("Goal sent: x = %f, y = %f", x, y)

    def cancel_goal(self):
        """
        \brief Cancels all goals currently queued or active in the action server.
        """
        self.client.cancel_all_goals()
        rospy.loginfo("Goals cancelled")

    def reset_world(self):
        """
        \brief Calls the service to reset the simulation world in Gazebo.
        """
        reset = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        try:
            reset()
            rospy.loginfo("Gazebo world reset successfully.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def obst_dist(self):
        """
        \brief Subscribes to the /scan topic to get laser scan data and compute obstacle distance.
        Needs implementation to process the data.
        """
        rospy.loginfo("Obstacle distance measurement functionality needs implementation.")

    def listen_goals(self):
        """
        \brief Continuously processes user inputs to dynamically control the robot through action server goals, resets, or obstacle checks.
        """
        while not rospy.is_shutdown():
            user_input = input("Enter a goal as 'x,y', type 's' to stop or 'r' to reset the environment and 'o' to know the distance of the obstacle to left:")
            if user_input == 's':
                self.cancel_goal()
            elif user_input == 'r':
                self.reset_world()
            elif user_input == 'o':
                self.obst_dist()
            else:
                try:
                    x, y = map(float, user_input.split(','))
                    self.send_goal(x, y)
                except ValueError:
                    rospy.loginfo("Invalid input. Please enter the goal as 'x,y' as numbers or type 'stop' to cancel.")

def main():
    """
    \brief Main function to initialize the ROS node and start the action client.
    """
    rospy.init_node('action_client', anonymous=True)
    client = ActionClient()
    goal_thread = threading.Thread(target=client.listen_goals)
    goal_thread.start()
    rospy.spin()
    goal_thread.join()

if __name__ == '__main__':
    main()

