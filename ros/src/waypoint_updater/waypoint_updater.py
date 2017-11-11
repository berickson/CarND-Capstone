#!/usr/bin/env python

import rospy
import std_msgs
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import sys
import math
import numpy as np
import copy
from scipy import spatial

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

LOOKAHEAD_WPS = 100 #0 # Number of waypoints we will publish. You can change this number

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
        rospy.Subscriber('/traffic_waypoint', std_msgs.msg.Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        self.lane = None
        self.closest_waypoint_index = None
        self.lookahead_wp_count = 0
        self.waypoint_count = 0
        self.red_light_waypoint_number = -1
        self.last_update_time = 0.0
        self.update_period = 0.2 # seconds


        rospy.spin()

    def pose_cb(self, pose):
        '''
        pose: geometry_msgs/PoseStamped
        '''
        t = rospy.get_time()
        if t - self.last_update_time < self.update_period : return
        self.last_update_time = t
        if self.lane and self.kd_tree:
            # find closest waypoint
            all_d,all_i = self.kd_tree.query([(pose.pose.position.x, pose.pose.position.y)])
            self.closest_waypoint_index = all_i[0]

            
            lane = Lane()

            if self.closest_waypoint_index + self.lookahead_wp_count +1 > self.waypoint_count:
                last_index = self.closest_waypoint_index + self.lookahead_wp_count + 1 - self.waypoint_count
                lane.waypoints =  copy.deepcopy(self.lane.waypoints[self.closest_waypoint_index:] + self.lane.waypoints[:last_index])
            else:
                lane.waypoints = copy.deepcopy(self.lane.waypoints[self.closest_waypoint_index: self.closest_waypoint_index + self.lookahead_wp_count +1])

            # apply global max velocity
            global_max_velocity = rospy.get_param('/waypoint_loader/velocity')
            for wp in lane.waypoints:
                wp.twist.twist.linear.x = min(wp.twist.twist.linear.x, global_max_velocity)

            if self.red_light_waypoint_number:

                wp_red = self.red_light_waypoint_number - self.closest_waypoint_index
                if wp_red >= 0 and wp_red < len(lane.waypoints):
                    rospy.loginfo("stopping for red light")
                    for i in range(len(lane.waypoints)):
                        # set maximum speed at each point to ensure we stop at end
                        # todo: what about wrap around?
                        if i >= wp_red:
                            lane.waypoints[i].twist.twist.linear.x = 0.0
                            lane.waypoints[i].twist.twist.linear.y = 0.0
                            lane.waypoints[i].twist.twist.linear.z = 0.0
                        else:
                            stop_ahead = 8
                            a = 2
                            s = self.distance(lane.waypoints, i, wp_red)
                            s = max(s-stop_ahead,0)
                            max_v = max(math.sqrt(2*a*s),0)
                            if lane.waypoints[i].twist.twist.linear.x > max_v:
                                lane.waypoints[i].twist.twist.linear.x = max_v

            self.final_waypoints_pub.publish(lane)

    def waypoints_cb(self, lane):
        '''
        receives a single message from /base_waypoints with all of the waypoints for the map
        '''
        self.lane = lane

        self.kd_tree = spatial.KDTree([[wp.pose.pose.position.x,wp.pose.pose.position.y] for wp in lane.waypoints ])
        
        self.waypoint_count = np.shape(lane.waypoints)[0]
        self.lookahead_wp_count = min(LOOKAHEAD_WPS, self.waypoint_count)

    def traffic_cb(self, msg):
        # Callback for /traffic_waypoint message.
        self.red_light_waypoint_number = msg.data

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
