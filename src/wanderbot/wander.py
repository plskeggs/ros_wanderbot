#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def scan_callback(msg):
    global g_range_ahead
    g_range_ahead = msg.ranges[0]
    if g_range_ahead > 10:
        g_range_ahead = 10


g_range_ahead = 1
driving_forward = False

scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

rospy.init_node('wander')
state_change_time = rospy.Time.now()
print "Now:", state_change_time
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    if driving_forward:
        if (g_range_ahead < 0.8 or rospy.Time.now() > state_change_time):
            driving_forward = False
            print "Starting to turn; range ahead:", g_range_ahead
            state_change_time = rospy.Time.now() + rospy.Duration(1)
    else:
        if (g_range_ahead > 1.0 and rospy.Time.now() > state_change_time):
            driving_forward = True
            print "Starting forward"
            state_change_time = rospy.Time.now() + rospy.Duration(30)
    twist = Twist()
    if driving_forward:
        twist.linear.x = g_range_ahead / 4
        twist.angular.z = 0.1
    else:
        twist.angular.z = 0.5
    print "lin %0.1f, ang %0.1f" % (twist.linear.x, twist.angular.z)
    cmd_vel_pub.publish(twist)

    rate.sleep()



