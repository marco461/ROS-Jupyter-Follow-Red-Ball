# ROS Jupyter Follow Red Ball

This project contains a ROS (Robot Operating System) node written in Python that enables a robot equipped with a camera to detect and follow a red ball. The code processes RGB and depth images, identifies red balls, and sends commands to the robot to follow the detected ball.

## Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/marco461/jupyter-follow-red-ball.git
   cd jupyter-follow-red-ball
   ```

2. **Install dependencies:**
   Ensure that you have ROS and the required Python packages installed:
   ```bash
   sudo apt-get install ros-<ros-distro>-cv-bridge ros-<ros-distro>-image-transport
   pip install opencv-python numpy
   ```

## Usage

1. **Launch the ROS master node:**
   ```bash
   roscore
   ```

2. **Run the node:**
   ```bash
   python main.py
   ```

## Node Details

### Subscribed Topics

- `/camera_top/rgb/image_raw` (`sensor_msgs/Image`): The RGB image feed from the camera.
- `/camera_top/depth/image_raw` (`sensor_msgs/Image`): The depth image feed from the camera.

### Published Topics

- `/cmd_vel_mux/input/navi` (`geometry_msgs/Twist`): The velocity command for controlling the robot.

### Functionality

- **RGB Image Processing:** The `rgb_image_callback` function converts the incoming RGB image to OpenCV format, detects red balls, displays the processed image, and calls `control_robot` to send movement commands.
- **Depth Image Processing:** The `depth_image_callback` function converts the incoming depth image to OpenCV format and stores it for further processing.
- **Red Ball Detection:** The `detect_red_balls` function detects red balls in the RGB image using HSV color space thresholding and contour detection.
- **Robot Control:** The `control_robot` function sends velocity commands to the robot based on the position of the detected red ball relative to the center of the image.

### Code Overview

```python
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Twist

bridge = CvBridge()
depth_image = None
detected_objects = []

def rgb_image_callback(data):
    try:
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
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
    mask = mask1 + mask2
    contours = []
    if int(cv2.__version__.split('.')[0]) >= 4:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    else:
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detected_objects = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if w > 20 and h > 20:
            (x_center, y_center), radius = cv2.minEnclosingCircle(contour)
            area = cv2.contourArea(contour)
            circularity = 4 * np.pi * (area / (2 * np.pi * radius)**2)
            if circularity > 0.8:
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
        move_cmd = Twist()
        cmd_vel_pub.publish(move_cmd)
        return
    closest_ball = min(detected_objects, key=lambda obj: depth_image[obj[1], obj[0]])
    obj_x = closest_ball[0] - depth_image.shape[1] // 2
    move_cmd = Twist()
    if obj_x == 0:
        move_cmd.angular.z = 0.1
        move_cmd.linear.x = 0
    else:
        if -40 <= obj_x <= 30:
            move_cmd.angular.z = 0
            move_cmd.linear.x = 0.3
        elif obj_x > 30:
            move_cmd.angular.z = -0.4
            move_cmd.linear.x = 0
        elif obj_x < -30:
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
```

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.
