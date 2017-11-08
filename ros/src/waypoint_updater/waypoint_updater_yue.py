#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import sys
import math
import numpy as np

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

	self.lane = None
	self.waypoints = None
	self.current_pose = None
	self.current_waypoint_index = None
	self.lookahead_wps = 0
        
	rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
	self.current_pose = msg.pose

	#rospy.logwarn("Current_pose:\n%s", self.current_pose)


	if self.waypoints:
		self.current_waypoint_index = self.find_closest_waypoint(self.current_pose, self.waypoints)

		rospy.logwarn("WaypointUpdater: Car position index: %s", self.current_waypoint_index)

		#Find a fixed number of waypoints currently ahead of the vehicle
		if self.current_waypoint_index + self.lookahead_wps +1 > self.waypoints_number:
			last_index = self.current_waypoint_index + self.lookahead_wps + 1 - self.waypoints_number
		        self.final_waypoints = self.waypoints[self.current_waypoint_index:] + self.waypoints.waypoints[:last_index]
		else:
		        self.final_waypoints = self.waypoints[self.current_waypoint_index: self.current_waypoint_index + self.lookahead_wps +1]
	
		#rospy.logwarn("WaypointUpdater: final_waypoints: %s", self.final_waypoints[len(self.final_waypoints)-1])

		self.publish()

        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
	rospy.logwarn('logwarn WaypointUpdater: Got initial waypoints')
        if self.waypoints is None:
		self.lane = waypoints
        	self.waypoints = waypoints.waypoints
		rospy.logwarn("WaypointUpdater: Number of way points: %s", len(self.waypoints))

	self.waypoints_number = np.shape(waypoints.waypoints)[0]
        self.lookahead_wps = min(LOOKAHEAD_WPS, self.waypoints_number)

	pass

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

    def find_closest_waypoint(self, pose, waypoints):
	if waypoints is None or pose is None:
		return -1

	dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 )	

        position = pose.position
	min_distance = sys.float_info.max
	min_index = None

	for index, waypoint in enumerate(waypoints):
		distance = dl(waypoint.pose.pose.position, position)
		if distance < min_distance:
			min_distance = distance
			min_index = index

	return min_index

    def publish(self):

        lane = Lane()
        lane.header = self.lane.header
        lane.waypoints = self.final_waypoints

        self.final_waypoints_pub.publish(lane)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
