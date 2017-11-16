import tensorflow as tf
import cv2
import numpy as np
import rospy

from styx_msgs.msg import TrafficLight


class TLClassifier(object):
    def __init__(self):
        #TODO load classifier

        PATH_TO_MODEL = '../../../models/frozen_inference_graph_site.pb'
        self.detection_graph = tf.Graph()

        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()

            # Works up to here.
            with tf.gfile.GFile(PATH_TO_MODEL, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_detection = self.detection_graph.get_tensor_by_name('num_detections:0')
	    self.sess = tf.Session(graph=self.detection_graph)

        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

	# Bounding Box Detection.
	with self.detection_graph.as_default():
	# Expand dimension since the model expects image to have shape [1, None, None, 3].
		img_expanded = np.expand_dims(image, axis=0)  
		(boxes, scores, classes, num) = self.sess.run(
						[self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detection],
						feed_dict={self.image_tensor: img_expanded})
	
	return self.get_most_probable_state(scores[0], classes[0])

    def get_most_probable_state(self, scores, classes):
	state_num = 4
	detection_thresold = 0.5
	most_probable_state = 4
	highest_score = 0
	for i in range(state_num):
		if scores[i] > detection_thresold and scores[i] > highest_score:
			#rospy.logwarn("scores:\n%s", scores[i])
			highest_score = scores[i]
			most_probable_state = classes[i]-1

	return int(most_probable_state)
