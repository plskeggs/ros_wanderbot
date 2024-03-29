#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    range_ahead = msg.ranges[0]
    closest_range = min(msg.ranges)
    print "range ahead: %0.1f, closest range: %0.1f" % (range_ahead, closest_range)

rospy.init_node('range_ahead')
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
rospy.spin()


