from styx_msgs.msg import TrafficLight
import tensorflow as tf
import cv2
import numpy as np
import rospy

class TLClassifier(object):
    def __init__(self):
	self.sim_enabled = True
        if self.sim_enabled:
            model_name = '../../../models/frozen_inference_graph_sim.pb'
        else:
            model_name = '../../../models/frozen_inference_graph_real.pb'

        self.image_np_deep = None
        self.detection_graph = tf.Graph()

        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True

        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(model_name, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            self.sess = tf.Session(graph=self.detection_graph, config=config)

        # Definite input and output Tensors for detection_graph
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')


    def get_classification(self, image):
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image, axis=0)

        # Actual detection.
        (boxes, scores, classes, num) = self.sess.run(
            [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
            feed_dict={self.image_tensor: image_np_expanded})

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        if len(scores) < 1:
            return TrafficLight.UNKNOWN;

        return TLClassifier.get_most_probable_state(scores[0], classes[0])


    @staticmethod
    def get_most_probable_state(scores, classes):
	detection_threshold = 0.5

	if scores > detection_threshold:
		if classes == 1:
		    return TrafficLight.GREEN
		elif classes == 2:
		    return TrafficLight.RED
		elif classes == 3:
		    return TrafficLight.YELLOW
		return TrafficLight.UNKNOWN
        else:
            return TrafficLight.UNKNOWN
