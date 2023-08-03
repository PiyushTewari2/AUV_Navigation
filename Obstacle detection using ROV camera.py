#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
import cv2
import numpy as np

# Load YOLO
net = cv2.dnn.readNet("/home/cmeri/darknet/yolov3.weights", "/home/cmeri/darknet/cfg/yolov3.cfg")
classes = []
with open("/home/cmeri/darknet/cfg/coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

# Set CUDA as the preferred backend and target for better performance
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

# Get output layer names
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

cap = cv2.VideoCapture(0)   #0 specify the camera index, i.e the camera used

# Initialize ROS node and publisher
rospy.init_node('object_camera')   
pub = rospy.Publisher('object_camera', Int32, queue_size=10)

# Read frame from webcam
ret, frame = cap.read()

# Resize and convert the frame to a blob
blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)

# Set the input to the network
net.setInput(blob)

# Run forward pass through the network
outs = net.forward(output_layers)

# Initialize variables
class_ids = []
confidences = []
boxes = []
height, width, channels = frame.shape

# Process each output layer
for out in outs:
    for detection in out:
        scores = detection[5:]
        class_id = np.argmax(scores)
        confidence = scores[class_id]

        # Filter out weak predictions
        if confidence > 0.5:
            center_x = int(detection[0] * width)
            center_y = int(detection[1] * height)
            w = int(detection[2] * width)
            h = int(detection[3] * height)

            # Rectangle coordinates
            x = int(center_x - w / 2)
            y = int(center_y - h / 2)

            boxes.append([x, y, w, h])
            confidences.append(float(confidence))
            class_ids.append(class_id)

# Perform non-maximum suppression to eliminate redundant overlapping boxes
indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
    
object_detected = 1 if len(indexes) > 0 else 0

# Publish object detection value using ROS publisher
pub.publish(object_detected)

# Draw bounding boxes and labels on the original frame
for i in range(len(boxes)):
    if i in indexes:
        x, y, w, h = boxes[i]
        label = str(classes[class_ids[i]])
        confidence = confidences[i]

        # Draw bounding box and label
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, "{}: {:.2f}".format(label, confidence), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

rospy.spin()
