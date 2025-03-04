# Image Converter Node

This ROS node subscribes to semantic (color) images, depth images, and camera intrinsic parameters, then processes these inputs to detect a particular colored object and estimate its 3D position in the world frame. The node includes optional publishing of visualization markers to RViz for debugging.

## Overview

1.**Semantic Image Processing**

- Subscribes to a topic of color (semantic) images.
- Converts the image to grayscale and performs thresholding to isolate a specific color intensity (tuned for a yellow-like value).
- Uses morphological operations to filter noise.
- Identifies contours to locate objects of interest.
- Extracts the object's bounding box and its centroid.

2.**Depth Image Processing**

- Subscribes to a depth image topic.
- Takes the previously determined 2D centroid and retrieves the corresponding depth (in millimeters).
- Converts this 2D point with depth data into a 3D point in the camera coordinate frame using the camera intrinsic matrix.

3.**Coordinate Transformation**

- Uses TF2 to transform the camera-frame 3D point into the "world" frame.
- Verifies the availability of a transform before converting the point.

4.**Publishing**

- Optionally publishes the transformed 3D point.
- Publishes a visualization marker (a sphere) in RViz to illustrate the detected object's position.

## Subscribed Topics

-`/unity_ros/Quadrotor/Sensors/SemanticCamera/image_raw` (sensor_msgs/Image)

  Provides the semantic color images for object detection.

-`/realsense/depth/image` (sensor_msgs/Image)

  Supplies depth information (16-bit encoding).

-`realsense/depth/camera_info` (sensor_msgs/CameraInfo)

  Delivers intrinsic camera parameters used for 2D-to-3D conversion.

## Published Topics

-`point/latern` (geometry_msgs/PointStamped)

  Optional publication of the object’s 3D position in the "Quadrotor/Sensors/DepthCamera" frame, transformed to the "world" frame (commented out by default).

-`visualization_marker` (visualization_msgs/Marker)

  RViz marker for visualizing the object’s location in the “world” coordinate frame.

## Node Name

-**image_converter**

  Defined in the main function as `ros::init(argc, argv, "image_converter");`.

## How It Works

1.**Semantic Image Callback** (`onSemImg`)

   Detects objects by thresholding a critical pixel intensity in grayscale. Morphological operations refine the mask, and valid contours give bounding boxes. The bounding box's centroid is highlighted for subsequent depth lookup.

2.**Depth Image Callback** (`onDepthImg`)

   Retrieves the depth at the centroid. Converts pixel coordinates and depth to camera coordinates, then uses TF2 to transform the point to "world." Optionally publishes the point and a 3D marker.

3.**Camera Info Callback** (`onDepthInfo`)

   Updates the camera's intrinsic matrix and timestamp used for consistent time alignment.

## Requirements

- ROS (tested with melodic/noetic or comparable version).
- OpenCV for image processing (including highgui and imgproc).
- cv_bridge for bridging ROS and OpenCV.
- TF2 for coordinate transformations.

## Usage

1. Ensure that the necessary topics:

- Semantic image (`/unity_ros/Quadrotor/Sensors/SemanticCamera/image_raw`)
- Depth image (`/realsense/depth/image`)
- Camera info (`/realsense/depth/camera_info`)

  are being published by your system.

2. Build and run this node:

   ```bash
   catkin build
   rosrun image_converter_node sem_img_proc_node
   ```
3. In RViz (optional), select the “visualization_marker” topic from this node to visualize the 3D marker corresponding to the detected object in the “world” frame.
