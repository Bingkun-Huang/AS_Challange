# FrontierDetecter

## Overview
`FrontierDetecter` is responsible for generating autonomous exploration waypoints for a UAV using depth images from a RealSense camera. The node analyzes the depth image to detect navigable free space and publishes the next waypoint for the UAV.

---

## Features
- Subscribes to depth images and camera calibration data.
- Processes depth images to identify the largest connected free-space region (frontier).
- Converts the detected frontier center from image coordinates to 3D world coordinates.
- Publishes the next waypoint to `/point/correction`.
- Monitors the waypoint queue status and triggers new waypoint generation when needed.
- Handles fallback scenarios when no valid frontier is detected (publishes a small offset point).
- Optionally detects if the UAV reaches the cave exit and publishes a final point.

---

## Topics

### Subscribed Topics
| Topic | Type | Description |
|---|---|---|
| `/realsense/depth/image` | `sensor_msgs/Image` | Depth image from RealSense camera. |
| `/realsense/depth/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsic parameters. |
| `/current_state_est` | `nav_msgs/Odometry` | Current UAV position and orientation. |
| `/state/waypoints_queue` | `std_msgs/Bool` | Flag indicating whether the waypoint queue is empty. |
| `/length_his_queue` | `std_msgs/Int64` | Length of the waypoint history queue. |



## Key Parameters
| Parameter | Default | Description |
|---|---|---|
| `voxel_size` | 1.0 | (Optional) Scale for converting depth map to 3D coordinates. |
| `depth_in_meter` | 36.0 | Assumed depth used for target point projection. |
| `fallback_offset` | 0.1 | Offset distance applied if no valid frontier is detected. |

---

## Data Flow Diagram
```text
+-------------------------+
| RealSense Depth Camera  |
+-------------------------+
            |
    /realsense/depth/image
            |
+----------------------+
| FrontierDetecter     |
| (Waypoint Generator) |
+----------------------+
            |
    /point/correction
            |
+------------------+
| Path Planner    |
| (A*
+------------------+
```
