
# Image Converter Node

## Overview

The **Image Converter Node** processes semantic (RGB) and depth images to detect a specific object (e.g., a lantern) in a UAV system. It extracts the object’s pixel coordinates from the semantic image using thresholding and morphological operations, retrieves the corresponding depth value from the depth image, converts the 2D image coordinates into a 3D point in the camera frame using intrinsic parameters, and finally transforms this 3D point into the world coordinate frame using TF2. The node then publishes the detected position as a `geometry_msgs/PointStamped` message and also visualizes it in RViz using a marker.

## Algorithm Overview

1. **Semantic Image Processing**

   - **Image Conversion**: Converts the ROS image (RGB) into an OpenCV BGR image.
   - **Grayscale & Thresholding**: Converts to grayscale and applies a fixed intensity threshold to isolate the object of interest.
   - **Morphological Operations**: Uses dilation and closing to reduce noise and merge close regions.
   - **Contour Detection & Centroid**: Finds contours in the binary mask, selects the largest one, and calculates the bounding box centroid.
2. **Depth Image Processing**

   - **Depth Extraction**: Retrieves the depth value (in millimeters) at the centroid and converts it to meters.
   - **3D Conversion**: Forms a homogeneous coordinate vector and multiplies by the inverse of the camera intrinsic matrix to get camera-frame 3D coordinates.
3. **Coordinate Transformation**

   - **TF Transform**: Transforms the 3D point from the camera coordinate frame to the world frame using TF2.
4. **Publishing**

   - **PointStamped**: Publishes the final 3D position in the world frame.
   - **Marker**: Optionally publishes a visualization marker to help visualize the point in RViz.

---

## Subscribed Topics

| **Topic Name**                                      | **Message Type**     | **Description**                         |
| --------------------------------------------------------- | -------------------------- | --------------------------------------------- |
| `/unity_ros/Quadrotor/Sensors/SemanticCamera/image_raw` | `sensor_msgs/Image`      | Semantic image input from a simulated camera. |
| `/realsense/depth/image`                                | `sensor_msgs/Image`      | Depth image input from a depth camera.        |
| `/realsense/depth/camera_info`                          | `sensor_msgs/CameraInfo` | Camera intrinsic parameters.                  |

---

## Published Topics

| **Topic Name**     | **Message Type**         | **Description**                               |
| ------------------------ | ------------------------------ | --------------------------------------------------- |
| `point/latern`         | `geometry_msgs/PointStamped` | The detected lantern's position in the world frame. |
| `visualization_marker` | `visualization_msgs/Marker`  | Visualization marker for RViz display.              |

---

## Node Details

| **Component**      | **Description**                                                                                         |
| ------------------------ | ------------------------------------------------------------------------------------------------------------- |
| `ImageConverter` Class | Handles image processing, depth extraction, coordinate transformation, and marker publishing.                 |
| `onSemImg()`           | Processes the semantic (RGB) image, detects the object, and extracts its centroid.                            |
| `onDepthImg()`         | Retrieves the depth value at the centroid, converts 2D coordinates to 3D, and transforms them to world frame. |
| `onDepthInfo()`        | Retrieves camera intrinsic parameters from the camera info message.                                           |

---

## Dependencies

This node relies on the following ROS packages and external libraries:

- **ROS Packages**:

  - `roscpp`
  - `image_transport`
  - `cv_bridge`
  - `sensor_msgs`
  - `tf2_ros`
  - `geometry_msgs`
  - `visualization_msgs`
- **Libraries**:

  - **OpenCV**
  - **Eigen** (if needed for linear algebra operations)

---

## How to Run

1. **Build the Package**Place this package in your Catkin workspace (e.g., `~/catkin_ws/src/image_converter_node`), then compile:

   ```bash
   cd ~/catkin_ws
   catkin build
   source devel/setup.bash
   ```
2. **Launch the Node**

   ```bash
   rosrun image_converter_node sem_img_proc_node
   ```
3. **RViz Setup**

   - Set **Fixed Frame** to `world`.
   - Add a **Marker** display subscribing to `/visualization_marker`.
   - (Optionally) add a **Point** or **PointStamped** display for `point/latern`.

---


## Test Video

```bash
https://drive.google.com/drive/folders/1NU2AQS5s1sfsz31FieOUfe_J1ipO6Qpv?usp=sharing
```


---

## Summary

The **Image Converter Node** integrates semantic image processing and depth image analysis to provide accurate 3D localization of an object in the UAV’s environment. Its outputs (both as a `PointStamped` message and a marker in RViz) are useful for navigation, object tracking, or higher-level decision-making tasks in a mapped environment.
