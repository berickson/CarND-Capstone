from styx_msgs.msg import TrafficLight
import numpy as np
import cv2

from keras.models import model_from_json
import tensorflow as tf
import keras
from keras.models import load_model

import json
import rospy

from keras.utils.generic_utils import CustomObjectScope

class TLClassifier(object):

    def __init__(self):
        self.SIM = True
        
        # real
        if(not self.SIM):
            model_path = '../../../models/model_real.json'
            weights_path = '../../../models/model_real.hdf5'
            self.image_size = (224, 224)
            
            rospy.loginfo("Real Model Path Set")
        
        # sim
        if(self.SIM):
            model_path = '../../../models/model_sim.json'
            weights_path = '../../../models/model_sim.hdf5'
            self.image_size = (128, 128)
            
            rospy.loginfo("SIM Model Path Set")
        
        
        with CustomObjectScope({'relu6': keras.applications.mobilenet.relu6,'DepthwiseConv2D': keras.applications.mobilenet.DepthwiseConv2D}):            
            with open(model_path, 'r') as f:
                loaded_model_json = f.read()
                
            self.model = model_from_json(loaded_model_json)
            
        self.model.load_weights(weights_path, by_name=True)
        self.graph = tf.get_default_graph()
        
        rospy.loginfo("Model Loaded")

    def get_classification(self, image):
        """
        Determines the color of the traffic light in image

        Args:
        image (cv::Mat): image containing the traffic light

        Returns:
        int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        if(not self.SIM):
            image = cv2.resize(image, self.image_size, interpolation=cv2.INTER_CUBIC)
        
            with self.graph.as_default():
                result = self.model.predict(image[None, :, :, :], batch_size=1)
                
            prediction = np.argmax(result)
        
            if prediction == 0:
                state = TrafficLight.RED
            elif prediction == 1:
                state = TrafficLight.YELLOW
            elif prediction == 2:
                state = TrafficLight.GREEN
            else:
                state = TrafficLight.UNKNOWN
        
        if(self.SIM):
            image = cv2.resize(image, self.image_size, interpolation=cv2.INTER_LINEAR)
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
