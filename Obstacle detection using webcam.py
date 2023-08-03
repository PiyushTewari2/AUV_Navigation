#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectCamera:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.object_pub = rospy.Publisher('/object_detection', Int32, queue_size=10)
        self.yolov3_model = None  # Placeholder for the YOLOv3 model

    def load_yolov3_model(self):
        # Load YOLOv3 weights and configuration
        weights_path = '/home/cmeri/darknet/yolov3.weights'
        config_path = '/home/cmeri/darknet/cfg/yolov3.cfg'
        coco_names_path = '/home/cmeri/darknet/cfg/coco.names'

        # Read the weights and configuration files
        net = cv2.dnn.readNetFromDarknet(config_path, weights_path)

        # Read the class names from the file
        with open(coco_names_path, 'r') as f:
            classes = [line.strip() for line in f.readlines()]

        # Set CUDA as the preferable backend and target
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

        # Get the layer names and output layers
        layer_names = net.getLayerNames()
        output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

        return net, classes, output_layers

    def image_callback(self, msg):
        try:
            # Convert the image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        if self.yolov3_model is None:
            # Load the YOLOv3 model on the first callback
            self.yolov3_model = self.load_yolov3_model()

        # Apply object detection using YOLOv3 on the image
        objects_detected = self.detect_objects(cv_image)

        # Publish 1 if objects are detected, otherwise publish 0
        if objects_detected:
            self.object_pub.publish(1)
        else:
            self.object_pub.publish(0)

        # Display the live video with object detection results
        cv2.imshow("Object Detection", cv_image)
        cv2.waitKey(1)

    def detect_objects(self, image):
        # Convert the image to a blob for input to the neural network
        blob = cv2.dnn.blobFromImage(image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)

        # Set the input to the YOLOv3 model
        self.yolov3_model[0].setInput(blob)

        # Run forward pass through the network
        outs = self.yolov3_model[0].forward(self.yolov3_model[2])

        # Initialize variables
        class_ids = []
        confidences = []
        boxes = []
        width = image.shape[1]
        height = image.shape[0]

        # Process each output layer
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]

                if confidence > 0.5:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    class_ids.append(class_id)
                    confidences.append(float(confidence))
                    boxes.append([x, y, w, h])

        # Perform non-maximum suppression to eliminate redundant overlapping boxes
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        # Draw bounding boxes and labels on the image
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = self.yolov3_model[1][class_ids[i]]

                # Draw bounding box and label
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Return True if objects are detected, otherwise False
        return len(boxes) > 0


if __name__ == '__main__':
    # Initialize the ROS node and create the ObjectCamera object
    rospy.init_node('object_camera', anonymous=True)
    object_camera = ObjectCamera()

    try:
        # Run the ROS node until interrupted
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    # Close all OpenCV windows
    cv2.destroyAllWindows()
