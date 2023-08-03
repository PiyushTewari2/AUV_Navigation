#!/usr/bin/env python

import rospy                                                         
from std_msgs.msg import String

th1 = th2 = th3 = th4 = th5 = th6 = 1500  # Default throttle values
joy_cam_tilt = 65  # Joystick value for camera tilt
joy_light = 65  # Joystick value for lights
count=0

rotation=150
forward=300

if __name__ == '__main__':
    # Initialize ROS node and publisher
    pub_mc = rospy.Publisher('command_MCstr_pub', String, queue_size=1)
    rospy.init_node('initial')
    rate = rospy.Rate(10)

    rospy.sleep(60)  # Wait for 60 seconds before starting control loop
    
    while not rospy.is_shutdown():
        if count % 6 == 0:
            th3 = 1500 - rotation
            th4 = 1500 + rotation
            th1 = th2 = th5 = th6 = 1500 
        else:
            th1 = th2 = 1500
            th3 = th4 = th5 = th6 = 1500 + forward    
        str_thlight = '$' + str(th1) + str(th2) + str(th3) + str(th4) + str(th5) + str(th6) + chr(joy_cam_tilt) + chr(joy_light) + '\n'  
        pub_mc.publish(str_thlight)
        count=count+1
        rate.sleep()
