#!/usr/bin/env python
import rospy
import actionlib
import math

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Pose, Point

waypoints = [
    [(6.5, 4.43, 0.0),(0.0, 0.0, -0.984047240305, 0.177907360295)],
    [(2.1, 2.2, 0.0), (0.0, 0.0, 0.0, 1.0)]
]
g_point = Point(0, 0, 0)

def feedback_cb(feedback):
	cur = feedback.base_position.pose.position
	distance = math.sqrt(math.pow(cur.x - g_point.x, 2.0) + math.pow(cur.y - g_point.y, 2.0) + math.pow(cur.z - g_point.z, 2.0))
	print('[Feedback] (%f, %f, %f) dist: %f' % (cur.x, cur.y, cur.z, distance))   

def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
    
    return goal_pose
    
if __name__ == '__main__':
    rospy.init_node('patrol')
    
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    
    while not rospy.is_shutdown():
        for pose in waypoints:
            goal = goal_pose(pose)
            g_point = goal.target_pose.pose.position
            print("going to goal: %f, %f, %f" % (g_point.x, g_point.y, g_point.z))
            client.send_goal(goal, feedback_cb = feedback_cb)
            client.wait_for_result()
            print('[Result]        State: %d' % (client.get_state()))
            print('[Result]       Status: %s' % (client.get_goal_status_text()))
