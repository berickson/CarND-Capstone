'''
tests for waypoint_updater
'''
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import waypoint_updater

def test_closest_waypoint_index():
    '''
    unit test for closest_waypoint_index
    '''
    lane = Lane()
    for x, y in [(0, 0), (1, 1.3), (2, 3)]:
        w = Waypoint()
        w.pose.pose.position.x = x
        w.pose.pose.position.y = y
        w.pose.pose.position.z = 0
        lane.waypoints.append(w)

    pose = PoseStamped()
    pose.pose.position.x = 0.9
    pose.pose.position.y = 0.95

    i = waypoint_updater.closest_waypoint_index(pose.pose, lane.waypoints)
    print('the closest waypoint was at index', i)

test_closest_waypoint_index()
