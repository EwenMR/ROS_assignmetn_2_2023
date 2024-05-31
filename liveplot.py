import matplotlib.pyplot as plt
import rospy
import tf
from turtlesim.msg import Pose
from tf.transformations import quaternion_matrix
import numpy as np
from matplotlib.animation import FuncAnimation

class Visualiser:
	def __init__(self):
		self.fig, self.ax = plt.subplots()
		self.ln, = plt.plot([], [], 'ro')
		self.x_data, self.y_data = [] , []

	def plot_init(self):
		self.ax.set_xlim(0, 10)
		self.ax.set_ylim(0, 10)
		return self.ln,
		
	def odom_callback(self, msg):
		self.y_data.append(msg.y)
		self.x_data.append(msg.x)

	def update_plot(self, frame):
		self.ln.set_data(self.x_data, self.y_data)
		return self.ln,
		
		
rospy.init_node('odom_visualizer_node')
vis = Visualiser()
sub = rospy.Subscriber('/turtle1/pose', Pose, vis.odom_callback)
ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.show(block=True)