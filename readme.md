# ImageConverter Node

## Overview
The `ImageConverter` node is a ROS-based implementation for detecting and localizing a specific object (a lantern) in an image stream. It processes both semantic and depth images to extract object position information and transform it into a global coordinate frame.

## Algorithm
The node follows these main steps:
1. **Semantic Image Processing**:
   - Subscribes to a semantic image topic.
   - Converts the received image to grayscale.
   - Applies a threshold to detect yellow objects (potential lanterns).
   - Uses morphological operations to refine the mask and extract contours.
   - Computes the bounding box and centroid of the detected object.
   - If a valid object is found, it marks its presence.

2. **Depth Image Processing**:
   - Subscribes to a depth image topic.
   - If the object was found in the semantic image, retrieves its depth value.
   - Converts the pixel coordinates and depth value into 3D camera coordinates.
   - Uses the camera intrinsic matrix to perform perspective projection.

3. **Coordinate Transformation**:
   - Uses `tf2_ros` to transform the detected 3D point from the camera frame to the world frame.
   - Publishes the transformed coordinates if a valid transformation is available.

## Subscribed Topics
| Topic Name | Message Type | Description |
|------------|-------------|-------------|
| `/unity_ros/Quadrotor/Sensors/SemanticCamera/image_raw` | `sensor_msgs/Image` | Semantic image input from a simulated camera. |
| `/realsense/depth/image` | `sensor_msgs/Image` | Depth image input from a depth camera. |
| `/realsense/depth/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsic parameters. |

## Published Topics
| Topic Name | Message Type | Description |
|------------|-------------|-------------|
| `point/latern` | `geometry_msgs/PointStamped` | The detected lantern's position in the world frame. |

## Node Details
| Component | Description |
|-----------|-------------|
| `ImageConverter` Class | Handles image processing, depth extraction, and coordinate transformation. |
| `onSemImg()` | Processes semantic images, detects objects, and extracts centroids. |
| `onDepthImg()` | Processes depth images, extracts depth values, and computes 3D coordinates. |
| `onDepthInfo()` | Retrieves camera intrinsic parameters. |

## Dependencies
This node relies on the following ROS packages and libraries:
- `roscpp`
- `image_transport`
- `cv_bridge`
- `sensor_msgs`
- `tf2_ros`
- `opencv`
- `Eigen`

## Running the Node
To run the node, make sure your ROS environment is set up correctly and launch the node:
```sh
rosrun your_package image_converter
```
Make sure that all necessary topics are being published before running the node.

