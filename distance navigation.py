#!/usr/bin/env python

import rospy                                       
from haversine import haversine                    
from haversine import inverse_haversine, Direction  
from numpy import arctan2, random, sin, cos, degrees, radians    
import math                                     
import numpy as np                                  
from sensor_msgs.msg import NavSatFix               
from std_msgs.msg import Float32                 
from std_msgs.msg import Int32                      
from std_msgs.msg import String			  

# Global variables
latitude = 0.0
longitude = 0.0
coord2 = (0.0, 0.0)
heading = 0
bearing = 0
th1 = th2 = th3 = th4 = th5 = th6 = 1500
joy_cam_tilt = 65
joy_light = 65
k_p = 1.7
threshold = 5
threshold2 = 60
count = 0
reached_flag = 0

def navSatFixCallback(msg): 
    """
    Callback function for NavSatFix message.
    Updates the current latitude and longitude values.
    """
    global latitude, longitude, coord2
    latitude = np.float64(msg.latitude)
    longitude = np.float64(msg.longitude)
    coord2 = (latitude, longitude)
    
def callback_heading(msg):
    """
    Callback function for heading message.
    Updates the current heading value.
    """
    global heading
    heading = msg.data
   
def anticlock(error):
    """
    Controls the thrusters in an anticlockwise direction based on the heading error.
    """
    global th1, th2, th3, th4, th5, th6
    th3 = int(1500 - k_p * error)
    th4 = int(1500 + k_p * error)
    th1 = th2 = th5 = th6 = 1500
    if error < threshold2:
        th3 = 1400
        th4 = 1600
    if error < threshold and error > -threshold:
        th3 = th4 = 1500
        
def clockwise(error):
    """
    Controls the thrusters in a clockwise direction based on the heading error.
    """
    global th1, th2, th3, th4, th5, th6
    th3 = int(1500 + k_p * error)
    th4 = int(1500 - k_p * error)
    th1 = th2 = th5 = th6 = 1500
    if error < threshold2:
        th3 = 1600
        th4 = 1400
    if error < threshold and error > -threshold:
        th3 = th4 = 1500
        
def heading_control(desired_heading, actual_heading):
    """
    Applies heading control based on the desired and actual heading.
    """
    global error
    if desired_heading < 180 and desired_heading >= 0:
        if actual_heading <= desired_heading + 180 and actual_heading >= desired_heading:  
            error = actual_heading - desired_heading
            anticlock(error)
        if actual_heading <= 360 and actual_heading >= desired_heading + 180:  
            error = desired_heading + 360 - actual_heading
            clockwise(error)
        if actual_heading <= desired_heading and actual_heading >= 0: 
            error = desired_heading - actual_heading
            clockwise(error)
    if desired_heading <= 360 and desired_heading >= 180:
        if actual_heading <= 360 and actual_heading >= desired_heading:   
            error = actual_heading - desired_heading
            anticlock(error)
        if actual_heading <= desired_heading - 180 and actual_heading >= 0:  
            error = actual_heading - desired_heading + 360
            anticlock(error)
        if actual_heading <= desired_heading and actual_heading >= desired_heading - 180:   
            error = desired_heading - actual_heading
            clockwise(error)

def coordinate_navigator(bear, final_coordinate):
    """
    Navigates the ROV to the final coordinate using bearing and distance control.
    """
    global th1, th2, th3, th4, th5, th6, reached_flag
    distance = haversine(coord2, final_coordinate)  # in kilometers
    distance = distance * 1000
    rospy.loginfo(distance)
    heading_control(bear, heading)
    if error < threshold and distance > 1:
        th1 = th2 = 1500
        th3 = th4 = th5 = th6 = 1800
        reached_flag = 0
    if error < threshold and distance < 1 and distance > 0.75:
        th1 = th2 = 1500
        th3 = th4 = th5 = th6 = 1650
        reached_flag = 0
    if error < threshold and distance < 0.75 and distance > 0.5:
        th1 = th2 = 1500
        th3 = th4 = th5 = th6 = 1600
        reached_flag = 0
    if distance < 0.5 and error < threshold:
        th1 = th2 = th3 = th4 = th5 = th6 = 1500
        reached_flag = 1

def bearing_calculator(end):
    """
    Calculates the bearing (heading) to reach the destination coordinate from the current coordinate.
    """
    global bearing
    lat1 = radians(coord2[0])
    long1 = radians(coord2[1])
    lat2 = radians(end[0])
    long2 = radians(end[1])
            
    dl = np.float64(long2 - long1)
    X = cos(lat2) * sin(dl)
    Y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dl)
    bearing = arctan2(X, Y)
    bearing = ((degrees(bearing) + 360) % 360)  # in degrees

if __name__ == '__main__':
    pub_mc = rospy.Publisher('command_MCstr_pub', String, queue_size=1)
    rospy.init_node('initial')
    rate = rospy.Rate(10)

    dis = input("Enter the length of desired location in meters: ")
    dis = float(dis)
    
    rospy.sleep(70)
    
    rospy.Subscriber("/an_device/Heading", Float32, callback_heading) 
    rospy.Subscriber("/an_device/NavSatFix", NavSatFix, navSatFixCallback)
    
    while not rospy.is_shutdown():
        if count == 2:   # getting the destination coordinate and initial coordinate
            initial_latitude = latitude 
            initial_longitude = longitude
            initial_heading = int(heading)
            
            initial_coordinate = (initial_latitude, initial_longitude)
            
            distance = np.float64(dis)  # in meters
            distance = distance / 1000
            
            destination_coordinate = inverse_haversine(initial_coordinate, distance, radians(initial_heading))
            
        if count >= 2:
            if reached_flag == 0:
                bearing_calculator(destination_coordinate)
                coordinate_navigator(bearing, destination_coordinate)

            if reached_flag == 1:
                th1 = th2 = th3 = th4 = th5 = th6 = 1500
                break
            str_thlight = '$' + str(th1) + str(th2) + str(th3) + str(th4) + str(th5) + str(th6) + chr(joy_cam_tilt) + chr(joy_light) + '\n'  
            pub_mc.publish(str_thlight)
        
        count += 1
        rate.sleep()
