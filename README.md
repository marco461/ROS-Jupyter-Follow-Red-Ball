# ROS Object Detection and Control

This repository contains a ROS node for detecting red balls in an RGB image stream, processing depth information, and controlling a robot based on the detected objects.

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Node Details](#node-details)
  - [Subscribing to RGB and Depth Images](#subscribing-to-rgb-and-depth-images)
  - [Detecting Red Balls](#detecting-red-balls)
  - [Displaying Images](#displaying-images)
  - [Controlling the Robot](#controlling-the-robot)
- [Example](#example)
- [License](#license)

## Installation

Ensure you have a working ROS environment and the necessary dependencies installed:

1. Clone the repository:
    ```bash
    git clone https://github.com/yourusername/ros-object-detection.git
    cd ros-object-detection
    ```

2. Install the dependencies:
    ```bash
    pip install opencv-python
    sudo apt-get install ros-<your_ros_distro>-cv-bridge ros-<your_ros_distro>-image-transport
    ```

3. Build the package:
    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

## Usage

Run the ROS node to start detecting red balls and controlling the robot:

```bash
rosrun ros_object_detection object_detector.py
```

## Node Details

This ROS node subscribes to RGB and depth image topics, processes the images to detect red balls, and publishes velocity commands to control the robot.

### Subscribing to RGB and Depth Images

The node subscribes to the following topics:
- `/camera_top/rgb/image_raw`: RGB image stream
- `/camera_top/depth/image_raw`: Depth image stream

```python
image_sub = rospy.Subscriber('/camera_top/rgb/image_raw', Image, rgb_image_callback)
depth_sub = rospy.Subscriber('/camera_top/depth/image_raw', Image, depth_image_callback)
```

### Detecting Red Balls

The `detect_red_balls` function processes the RGB image to detect red balls using HSV color space. It uses contours to identify circular shapes and returns the coordinates and radius of detected balls.

```python
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
```

### Displaying Images

The `display_images` function overlays the detected balls on the RGB image and displays it using OpenCV.

```python
def display_images(rgb_image, detected_objects):
    for (x_center, y_center, radius) in detected_objects:
        cv2.circle(rgb_image, (x_center, y_center), radius, (0, 255, 0), 2)

    cv2.imshow('RGB Image', rgb_image)
    cv2.waitKey(1)
```

### Controlling the Robot

The `control_robot` function publishes velocity commands to control the robot based on the detected red balls' positions.

```python
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
```

## Example

To run the node, simply execute the `object_detector.py` script. Ensure your ROS environment is correctly set up and the required topics are being published.

```bash
rosrun ros_object_detection object_detector.py
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
