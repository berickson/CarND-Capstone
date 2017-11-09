#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import sys
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 20 #00 # Number of waypoints we will publish. You can change this number

def closest_waypoint_index(pose, waypoints):
    '''
    returns the index of the waypoint closest to pose
    '''
    x = pose.position.x

    y = pose.position.y
    min_d2 = sys.float_info.max
    min_index = None
    for index, waypoint in enumerate(waypoints):
        p = waypoint.pose.pose.position
        dx = p.x - x
        dy = p.y - y
        d2 = dx * dx + dy * dy
        if d2 < min_d2:
            min_d2 = d2
            min_index = index
    return min_index



class WaypointUpdater(object):
    '''
    Planning module that listens to pose and publishes
    near term waypoints to be used by the control subsystems

    inputs:
        /current_pose
        /base_waypoints
    outputs:
        /final_waypoints
    '''
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        self.lane = None
        self.closest_waypoint_index = None
        self.lookahead_wps = 0
        self.waypoints_number = 0
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        rospy.spin()

    def pose_cb(self, pose):
        '''
        pose: geometry_msgs/PoseStamped

        '''
        if self.lane:
            self.closest_waypoint_index = closest_waypoint_index(pose.pose, self.lane.waypoints)
            lane = Lane()
            #lane.waypoints = self.lane.waypoints[self.closest_waypoint_index : self.closest_waypoint_index + LOOKAHEAD_WPS]
            
            if self.current_waypoint_index + self.lookahead_wps +1 > self.waypoints_number:
			    last_index = self.closest_waypoint_index + self.lookahead_wps + 1 - self.waypoints_number
		        self.final_waypoints = self.waypoints[self.closest_waypoint_index:] + self.waypoints.waypoints[:last_index]
		    else:
		        self.final_waypoints = self.waypoints[self.closest_waypoint_index: self.closest_waypoint_index + self.lookahead_wps +1]
            
            self.final_waypoints_pub.publish(lane)

    def waypoints_cb(self, lane):
        '''
        receives a single message from /base_waypoints with all of the waypoints for the map
        '''
        self.lane = lane
        
        self.waypoints_number = np.shape(lane.waypoints)[0]
        self.lookahead_wps = min(LOOKAHEAD_WPS, self.waypoints_number)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
