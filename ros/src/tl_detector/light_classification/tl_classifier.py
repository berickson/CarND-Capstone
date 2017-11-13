from styx_msgs.msg import TrafficLight
import numpy as np
import cv2
from keras.models import model_from_json
import tensorflow as tf
import json
import rospy


class TLClassifier(object):

    def __init__(self):
        with open('../../../models/model_sim.json', 'r') as f:
            loaded_model_json = f.read()

        self.model = model_from_json(loaded_model_json)
        self.model.load_weights('../../../models/model_sim.hdf5')
        self.graph = tf.get_default_graph()

    def get_classification(self, image):
        """
        Determines the color of the traffic light in image

        Args:
        image (cv::Mat): image containing the traffic light

        Returns:
        int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        image = cv2.resize(image, (128, 128), cv2.INTER_LINEAR)
        image = image.astype('float32') / 255
        image = image.reshape(1, *image.shape)
        
        with self.graph.as_default():
            predicted_class_id = self.model.predict_classes(image, batch_size=1, verbose=0)
            
        if predicted_class_id[0] == 0:
            state = TrafficLight.RED
        elif predicted_class_id[0] == 1:
            state = TrafficLight.YELLOW
        elif predicted_class_id[0] == 2:
            state = TrafficLight.GREEN
        else:
            state = TrafficLight.UNKNOWN

        return state
