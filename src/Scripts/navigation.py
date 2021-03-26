#!/usr/bin/env python
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
import rospy
from std_msgs.msg import Int32
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
import time
import roslaunch
import numpy as np
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from math import atan2
from geometry_msgs.msg import PoseArray
import tf.transformations
import csv

fp = open('path.csv', 'r')
lines=fp.readlines()
points=lines[0].split('"')
while (',') in points:
    points.remove(',')
points.remove('')
points.remove('\r\n')

for a in range(len(points)-1):
    points[a]=points[a].strip('][').split(', ')
    for b in range(3):
        points[a][b]=float(points[a][b])
points[-1]=points[-1].strip(')(').split(', ')
for b in range(3):
        points[-1][b]=float(points[-1][b])

print(points) #waypoints given by A*

follow = 1 #flag used for changing the next waypoint
flag = 'False'#flag used to switch between rotation and translation
def pose_callback(data):
	global x,y,flag,follow
	#getting self co-ordinates
	if follow<len(points):
		x = data.pose.pose.position.x
		y = data.pose.pose.position.y
		quartenion = (data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
		quarternion_transform = tf.transformations.euler_from_quaternion(quartenion)
		yaw = quarternion_transform[2]

		#setting goal co-ordinates
		goal_x = points[follow][0]
		goal_y = points[follow][1]
		goal_zrot = atan2((goal_y-y),(goal_x-x))

		# print("Current co-ord are",x,y,yaw)
		# print("Goal co-ord are",goal_x,goal_y,goal_zrot)

		dist2next = np.sqrt((x-(goal_x))**2+(y-goal_y)**2) #distance between current position and next waypoint

		vel_msg = Twist()
		pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
		ang_diff = goal_zrot-yaw #difference in angle pointing towards nextwaypoint and current angle
		# print("The ang diff is",ang_diff)
		if ang_diff>0.005:
			vel_msg.linear.x = 0
			vel_msg.angular.z = +0.4
			pub.publish(vel_msg)
		elif ang_diff<-0.005:
			vel_msg.linear.x = 0
			vel_msg.angular.z = -0.4
			pub.publish(vel_msg)
		else:
			vel_msg.angular.z = 0
			pub.publish(vel_msg)
			flag = 'True'
		if flag == 'True' and dist2next > 0.1:
			vel_msg.linear.x = 0.2
			vel_msg.angular.z = 0
			pub.publish(vel_msg)

		if dist2next <= 0.1:
			vel_msg.linear.x = 0
			pub.publish(vel_msg)
			follow += 1  #change the waypoint to next
			flag = 'False'


		#print(flag)
		#print("The distance to start is",dist2start)
		#print(follow)

	else: #stop at goal
		vel_msg = Twist()
		pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		pub.publish(vel_msg)
def listen_odom():
	global tag, launch_counter
	rospy.Subscriber("odom",Odometry, pose_callback)

	rospy.spin()

if __name__ == '__main__':
	try:
		rospy.loginfo("------INITIATING AUTONOMOUS CONTROLLER-------")
		rospy.init_node("navigation",anonymous=True)
		listen_odom()
	except rospy.ROSInterruptException: pass
