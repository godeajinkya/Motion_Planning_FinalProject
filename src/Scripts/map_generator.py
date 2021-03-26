#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid as OG
import math
import time
import numpy
import csv

rospy.sleep(30.)


def callback(points):
    map_data=points.data
    print("Map Data Captured")
    with open('mapdata.csv', 'w') as file:
        writer = csv.writer(file)
        writer.writerow(map_data)
    return map_data


rospy.init_node('map_gen', anonymous=True)
subscribe = rospy.Subscriber('map', OG, callback)
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    # publish.publish(velocity)

    rate.sleep()
