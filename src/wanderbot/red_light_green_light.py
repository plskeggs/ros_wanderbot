#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('red_light_green_light')

red_light_twist = Twist()
green_light_twist = Twist()
green_light_twist.linear.x = 0.5

driving_forward = False
while rospy.get_time() == 0:
    if rospy.is_shutdown():
        break
print 'Now:', rospy.Time.now()
light_change_time = rospy.Time.now() + rospy.Duration.from_sec(3)
rate = rospy.Rate(10)
print 'red light green light starting...'
print 'stopped'

while not rospy.is_shutdown():
    if driving_forward:
        cmd_vel_pub.publish(green_light_twist)
    else:
        cmd_vel_pub.publish(red_light_twist)
    if light_change_time > rospy.Time.now():
        driving_forward = not driving_forward
        if driving_forward:
            print 'forward'
        else:
            print 'stopped'
        light_change_time = rospy.Time.now() + rospy.Duration.from_sec(3)
    rate.sleep()
    print 'Now:', rospy.Time.now(), ' Next:', light_change_time

