from styx_msgs.msg import TrafficLight

import numpy as np
import cv2
import tensorflow as tf
import rospy

# Helper code
def load_image_into_numpy_array(image):
    return np.asarray(image, dtype="uint8")

def tl_light_classifier(image):
    '''
    manually classifies the traffic light colour
    '''
    width_to_trim = int(image.shape[1] / 3)
    height_to_trim = int(image.shape[0] / 8)
    sub_image = image[height_to_trim:(image.shape[0]-height_to_trim),
                      width_to_trim:(image.shape[1]-width_to_trim)]

    green_threshold = (200, 255)
    red_threshold = (200, 255)

    # colour space is RGB
    red = sub_image[:, :, 0]
    green = sub_image[:, :, 1]

    green_binary = np.zeros_like(green)
    green_binary[(green > green_threshold[0]) & (green <= green_threshold[1])] = 1

    red_binary = np.zeros_like(red)
    red_binary[(red > red_threshold[0]) & (red <= red_threshold[1])] = 1

    is_red = (np.sum(red_binary) / red_binary.size) > 0.05
    is_green = (np.sum(green_binary) / green_binary.size) > 0.05

    if is_red and is_green:
        return TrafficLight.YELLOW
    elif is_red:
        return TrafficLight.RED
    elif is_green:
        return TrafficLight.GREEN
    else:
        return TrafficLight.UNKNOWN

class TLClassifier(object):
    def __init__(self):
        MODEL_NAME = 'ssd_mobilenet_v1_coco_2017_11_17'

        #self.number_of_images = 0

        # Path to frozen detection graph. This is the actual model that is used for the object detection.
        PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

        # Load a (frozen) Tensorflow model into memory
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            self.sess = tf.Session(graph=self.detection_graph)
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
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # the array based representation of the image will be used later in order to prepare the
        # result image with boxes and labels on it.
        image_np = load_image_into_numpy_array(image)
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)

        with self.detection_graph.as_default():
            (boxes, scores, classes, num) = self.sess.run(
                [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
                feed_dict={self.image_tensor: image_np_expanded})

        boxes = np.squeeze(boxes)
        classes = np.squeeze(classes).astype(np.int32)
        scores = np.squeeze(scores)

        for idx, val in enumerate(classes):
            if val == 10:
                break

        # confidence threshold too low
        if scores[idx] < 0.1:
            return TrafficLight.UNKNOWN

        # rospy.loginfo('~~:idx: {}'.format(idx))

        nbox = boxes[idx]

        height = image.shape[0]
        width = image.shape[1]

        box = np.array([nbox[0]*height, nbox[1]*width, nbox[2]*height, nbox[3]*width]).astype(int)

        #rospy.loginfo('~~:box: {}'.format(box))

        tl_image = image[box[0]:box[2], box[1]:box[3]]
        #cv2.imwrite('tl_image_' + str(self.number_of_images) + '.jpg', tl_image)
        #self.number_of_images += 1

        return tl_light_classifier(tl_image)

