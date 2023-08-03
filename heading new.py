#!/usr/bin/env python

import rospy                                        
import math                                        
from std_msgs.msg import Float32                   
from std_msgs.msg import String

# Global variables for control parameters and thresholds
heading = 0
th1 = th2 = th3 = th4 = th5 = th6 = 1500  # Default throttle values
joy_cam_tilt = 65  # Joystick value for camera tilt
joy_light = 65  # Joystick value for lights
k_p = 1.7  # Proportional gain for heading control
threshold = 5  # Threshold for small heading error
threshold2 = 60  # Threshold for larger heading error

def callback_heading(msg):
    """
    Callback function for heading topic subscriber.
    Updates the global variable 'heading' with the received heading value.
    """
    global heading
    heading = msg.data
    
def anticlock(error):
    """
    Perform control actions for anticlockwise rotation.
    Adjusts the throttle values based on the heading error.
    """
    global th1, th2, th3, th4, th5, th6
    th3 = int(1500 - k_p * error)
    th4 = int(1500 + k_p * error)
    th1 = th2 = th5 = th6 = 1500
    if error < threshold2:
        th3 = 1400
        th4 = 1600
    if -threshold < error < threshold:
        th3 = th4 = 1500
        
def clockwise(error):
    """
    Perform control actions for clockwise rotation.
    Adjusts the throttle values based on the heading error.
    """
    global th1, th2, th3, th4, th5, th6
    th3 = int(1500 + k_p * error)
    th4 = int(1500 - k_p * error)
    th1 = th2 = th5 = th6 = 1500
    if error < threshold2:
        th3 = 1600
        th4 = 1400
    if -threshold < error < threshold:
        th3 = th4 = 1500
        
def heading_control(desired_heading, actual_heading):
    """
    Perform heading control based on desired and actual heading values.
    Determines the error between the desired and actual heading, and applies the appropriate control actions.
    """
    global error
    if 0 <= desired_heading < 180:
        if desired_heading <= actual_heading <= desired_heading + 180:
            error = actual_heading - desired_heading
            anticlock(error)
        #if 0 <= actual_heading <= desired_heading + 180:
        if desired_heading + 180 <= actual_heading <= 360:
            error = desired_heading + 360 - actual_heading
            clockwise(error)
        if 0 <= actual_heading <= desired_heading:
            error = desired_heading - actual_heading
            clockwise(error)
    if 180 <= desired_heading <= 360:
        if desired_heading <= actual_heading <= 360:
            error = actual_heading - desired_heading
            anticlock(error)
        if 0 <= actual_heading <= desired_heading - 180:
            error = actual_heading - desired_heading + 360
            anticlock(error)
        if desired_heading - 180 <= actual_heading <= desired_heading:
            error = desired_heading - actual_heading
            clockwise(error)

if __name__ == '__main__':
    # Initialize ROS node and publisher
    pub_mc = rospy.Publisher('command_MCstr_pub', String, queue_size=1)
    rospy.init_node('initial')
    rate = rospy.Rate(10)
    
    # Get desired heading from user input
    dh = input("Enter desired heading in degrees: ")
    dh = float(dh)
    rospy.sleep(60)  # Wait for 60 seconds before starting control loop
    rospy.Subscriber("/an_device/Heading", Float32, callback_heading)  # Subscribe to heading topic
    
    while not rospy.is_shutdown():
        # Perform heading control and publish throttle values
        heading_control(dh, heading)
        str_thlight = '$' + str(th1) + str(th2) + str(th3) + str(th4) + str(th5) + str(th6) + chr(joy_cam_tilt) + chr(joy_light) + '\n'  
        rospy.loginfo("str: %s", str_thlight)
        pub_mc.publish(str_thlight)
        rate.sleep()
