#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Twist

# Initialize the CvBridge
bridge = CvBridge()
depth_image = None
detected_objects = []

def rgb_image_callback(data):
    try:
        # Convert the ROS Image message to an OpenCV image
        rgb_image = bridge.imgmsg_to_cv2(data, 'bgr8')
        global detected_objects
        detected_objects = detect_red_balls(rgb_image)
        display_images(rgb_image, detected_objects)
        control_robot(detected_objects)
    except CvBridgeError as e:
        rospy.logerr(e)

def depth_image_callback(data):
    try:
        global depth_image
        depth_image = bridge.imgmsg_to_cv2(data, '32FC1')
    except CvBridgeError as e:
        rospy.logerr(e)

def detect_red_balls(rgb_image):
    # Convert the image to HSV color space
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    
    # Define the color range for detecting red color in HSV space
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
    
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
    
    # Combine the masks to cover the full red color range
    mask = mask1 + mask2
    
    # Find contours in the mask based on OpenCV version
    contours = []
    if int(cv2.__version__.split('.')[0]) >= 4:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    else:
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    detected_objects = []
    
    # Loop through the contours to find the circular ones
    for contour in contours:
        # Get the bounding rectangle for each contour
        x, y, w, h = cv2.boundingRect(contour)
        
        # Ensure the detected object is large enough
        if w > 20 and h > 20:
            # Fit a minimum enclosing circle around the contour
            (x_center, y_center), radius = cv2.minEnclosingCircle(contour)
            area = cv2.contourArea(contour)
            
            # Calculate circularity
            circularity = 4 * np.pi * (area / (2 * np.pi * radius)**2)
            
            # Check if the contour is circular enough
            if circularity > 0.8:  # Adjusted threshold for higher accuracy
                detected_objects.append((int(x_center), int(y_center), int(radius)))
    
    rospy.loginfo("Detected red balls: {}".format(detected_objects))
    
    return detected_objects

def display_images(rgb_image, detected_objects):
    for (x_center, y_center, radius) in detected_objects:
        cv2.circle(rgb_image, (x_center, y_center), radius, (0, 255, 0), 2)

    cv2.imshow('RGB Image', rgb_image)
    cv2.waitKey(1)

def control_robot(detected_objects):
    if not detected_objects:
        # Stop the robot if no red balls are detected
        move_cmd = Twist()
        cmd_vel_pub.publish(move_cmd)
        return
    
    # Find the closest red ball
    closest_ball = min(detected_objects, key=lambda obj: depth_image[obj[1], obj[0]])
    
    # Calculate the offset from the center of the image
    obj_x = closest_ball[0] - depth_image.shape[1] // 2

    # Calculate the control signals
    move_cmd = Twist()
    
    if obj_x == 0:
        # Rotate to search for the object
        move_cmd.angular.z = 0.1
        move_cmd.linear.x = 0
    else:
        if -40 <= obj_x <= 30:
            # Move straight
            move_cmd.angular.z = 0
            move_cmd.linear.x = 0.3
        elif obj_x > 30:
            # Turn left
            move_cmd.angular.z = -0.4
            move_cmd.linear.x = 0
        elif obj_x < -30:
            # Turn right
            move_cmd.angular.z = 0.4
            move_cmd.linear.x = 0

    cmd_vel_pub.publish(move_cmd)

def main():
    rospy.init_node('object_detector')
    image_sub = rospy.Subscriber('/camera_top/rgb/image_raw', Image, rgb_image_callback)
    depth_sub = rospy.Subscriber('/camera_top/depth/image_raw', Image, depth_image_callback)
    global cmd_vel_pub
    cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

