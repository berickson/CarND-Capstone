#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
#from light_classification.tl_classifier import TLClassifier
from light_classification.tl_classifier_yue import TLClassifier_yue
from light_classification.tl_classifier_ryein import TLClassifier_ryein
import tf
import cv2
import yaml
import math

class Point:
    def __init__( self, x=0, y=0):
        self.x = x
        self.y = y
    def __del__(self):
        class_name = self.__class__.__name__
      
LIGHT_LABELS = ['RED', 'YELLOW', 'GREEN', 'NONE', 'UNKNOWN']

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        
        self.bridge = CvBridge()
        #self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.model = rospy.get_param('~model_path',"")

        editor = rospy.get_param('~editor',"")

    	rospy.loginfo("model %s, editor %s", self.model, editor)

        if self.model == "None":
            self.light_classifier = None
        else:
            if editor == "yue":
                self.light_classifier = TLClassifier_yue(self.model)
            else:
                self.light_classifier = TLClassifier_ryein(self.model)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)


        
        self.has_image = False
        
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=1)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1, buff_size=2*52428800)


        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
      self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)
    
    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        last_distance = 10000
        return_index = 0

        #print(self.waypoints[0])

        for i, waypoint in enumerate(self.waypoints.waypoints):
          distance = self.distance(waypoint.pose.pose.position, pose)
          if(distance < last_distance):
            return_index = i
            last_distance = distance
            #print(distance)

        return return_index

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)
    
    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if (not (self.pose and self.waypoints)):
            return -1, TrafficLight.UNKNOWN

        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        car_position = self.get_closest_waypoint(self.pose.pose.position)

        #DONE find the closest visible traffic light (if one exists)

        light_wp = -1
        # look over all the stop light line and see if we are within our sight in the horizon
        for i, stop_line in enumerate(stop_line_positions):
            stop_line_point = Point(stop_line[0], stop_line[1])
            distance = self.distance(self.pose.pose.position, stop_line_point)
            # 100 seems like a good horizon line if it is less we will check further
            if(distance > 100):
                continue
            
            # once we get the stop light line we check for the closest waypoint
            stop_line_wp = self.get_closest_waypoint(stop_line_point)
            # is sthe stop light line point greater then the current car position?
            if (stop_line_wp > car_position):
                if(light_wp == -1 or light_wp > stop_line_wp):
                    light_wp = stop_line_wp
                    light = self.lights[i]
                    #print(light)

        # check if variable light is set
        if (light):
            #state = light.state # uncomment if need ground truth

            state = self.get_light_state(light) # uncomment for classifier

            rospy.loginfo("Predicted %s, Ground Truth %s", LIGHT_LABELS[state], LIGHT_LABELS[light.state])

            return light_wp, state
        
        # self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
