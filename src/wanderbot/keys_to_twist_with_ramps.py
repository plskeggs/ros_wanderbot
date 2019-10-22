#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist

key_mapping = { 'w': [ 0,  1],
                'x': [ 0, -1],
                'a': [-1,  0],
                'd': [ 1,  0],
                's': [ 0,  0] }
g_twist_pub = None
g_target_twist = None
g_last_twist = None
g_last_twist_send_time = None
g_vel_scales = [0.1, 0.1] # default to very slow
g_vel_ramps = [1, 1] # units: m/s^2

def ramped_vel(v_prev, v_target, t_prev, t_now, ramp_rate):
    # compute maximum velocity step
    step = ramp_rate * (t_now - t_prev).to_sec()
    sign = 1.0 if (v_target > v_prev) else -1.0
    error = math.fabs(v_target - v_prev)
    if error < step: # we can get there within this time step -- we're done
        return v_target
    else:
        return v_prev + sign * step # take a step towards the target velocity
        
def ramped_twist(prev, target, t_prev, t_now, ramps):
    tw = Twist()
    tw.angular.z = ramped_vel(prev.angular.z, target.angular.z, t_prev, t_now, ramps[0])
    tw.linear.x = ramped_vel(prev.linear.x, target.linear.x, t_prev, t_now, ramps[1])
    return tw
    
def send_twist():
    global g_last_twist_send_time, g_target_twist, g_last_twist, g_vel_scales, g_vel_ramps, g_twist_pub
    t_now = rospy.Time.now()
    g_last_twist = ramped_twist(g_last_twist, g_target_twist, g_last_twist_send_time, t_now, g_vel_ramps)
    g_last_twist_send_time = t_now
    g_twist_pub.publish(g_last_twist)    

def keys_cb(msg, twist_pub):
    global g_target_twist, g_last_twist, g_vel_scales
    if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
        return # unknown key
    vels = key_mapping[msg.data[0]]
    g_target_twist.angular.z = vels[0] * g_vel_scales[0]
    g_target_twist.linear.x = vels[1] * g_vel_scales[1]

def fetch_param(name, default):
    if rospy.has_param(name):
        print "%s set to %.1f" % (name, g_vel_scales[0])
        return rospy.get_param(name)
    else:
        rospy.logwarn("%s not provided; defaulting to %.1f" % (name, default))
        return default


if __name__ == '__main__':
    rospy.init_node("keys_to_twist")
    g_last_twist_send_time = rospy.Time.now()
    g_twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('keys', String, keys_cb, g_twist_pub)
    rate = rospy.Rate(20)
    g_last_twist = Twist()
    g_target_twist = Twist()
    print 'w = forward, x = backward'
    print 'a = spin cw; d = spin ccw'
    print 's = stop;  ctrl-c to quit'
    
    g_vel_scales[0] = fetch_param('~angular_scale', g_vel_scales[0])
    g_vel_scales[1] = fetch_param('~linear_scale', g_vel_scales[1])
    g_vel_ramps[0] = fetch_param('~angular_accel', g_vel_ramps[0])
    g_vel_ramps[1] = fetch_param('~linear_accel', g_vel_ramps[1])

    while not rospy.is_shutdown():
        send_twist()
        try:
            rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            break
    
    

