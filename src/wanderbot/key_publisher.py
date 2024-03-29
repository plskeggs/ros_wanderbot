#!/usr/bin/env python
import rospy, sys, select, tty, termios
from std_msgs.msg import String

if __name__ == '__main__':
    key_pub = rospy.Publisher('keys', String, queue_size=1)
    rospy.init_node("keyboard_driver")
    rate = rospy.Rate(100)
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    print "Publishing keystrokes.  Press Ctrl-C to exit..."

    while not rospy.is_shutdown():
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            key_pub.publish(sys.stdin.read(1))
        try:
            rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            break

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)


