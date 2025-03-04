#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <eigen3/Eigen/Dense>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

// The ImageConverter class handles subscribing to image topics, processing images,
// converting 2D image coordinates to 3D points, and publishing visualization markers.
class ImageConverter
{
  ros::NodeHandle nh_; // ROS node handle for communication

  // Subscribers and Publishers
  ros::Subscriber depth_info_sub_;   // Subscriber for camera depth information
  ros::Publisher point_pub_;           // (Optional) Publisher for 3D point messages
  image_transport::ImageTransport it_; // For handling image transport
  image_transport::Subscriber sem_img_sub_;   // Subscriber for semantic (color) images
  image_transport::Subscriber depth_img_sub_; // Subscriber for depth images
  image_transport::Publisher debug_image_pub_;  // (Optional) Publisher for debug images
  ros::Publisher marker_pub_;          // Publisher for visualization markers to RViz

  // Data members for image processing
  cv::Mat object_mask_;    // Binary mask representing the detected object
  cv::Point centroid;      // Centroid of the detected object's bounding box
  cv::Rect boundingBox;    // Bounding box of the detected object
  uint16_t centroid_depth; // Depth value at the centroid position
  bool is_light_found;     // Flag indicating whether the target object was detected

  cv::Mat dist_coeffs;     // Camera distortion coefficients (unused here but reserved)
  cv::Mat camera_matrix;   // Camera intrinsic parameters matrix

  ros::Time cur_time_stamp;  // Current timestamp (from camera info)

  // Persistent TF buffer and listener for coordinate transformation
  tf2_ros::Buffer tfBuffer_;             // TF buffer to store past transforms
  tf2_ros::TransformListener tfListener_; // TF listener that fills the tfBuffer_
  
  int marker_id_; // Counter used to assign unique IDs to each marker

public:
  // Constructor: initialize members, subscribers, and publishers.
  ImageConverter()
      : it_(nh_), 
        // Initialize the TF buffer with a long cache duration (3000 seconds)
        tfBuffer_(ros::Duration(3000)),
        tfListener_(tfBuffer_),
        marker_id_(0)  // Start marker IDs from 0
  {
      // Subscribe to semantic image topic from the camera
      sem_img_sub_ = it_.subscribe("/unity_ros/Quadrotor/Sensors/SemanticCamera/image_raw", 1,
                                   &ImageConverter::onSemImg, this);
      // Subscribe to depth image topic
      depth_img_sub_ = it_.subscribe("/realsense/depth/image", 1,
                                     &ImageConverter::onDepthImg, this);
      // Subscribe to camera info to receive camera intrinsic parameters
      depth_info_sub_ = nh_.subscribe("realsense/depth/camera_info", 1,
                                      &ImageConverter::onDepthInfo, this);
      // Advertise a topic for the 3D point (if needed)
      point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("point/latern", 10);
      // Advertise a topic for RViz visualization markers.
      marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

      // Create a debug window (using OpenCV) to display processed images.
      cv::namedWindow("Debug Image Window", cv::WINDOW_NORMAL);
      cv::resizeWindow("Debug Image Window", 640, 480);
  }

  // Destructor: clean up any resources used.
  ~ImageConverter()
  {
      cv::destroyWindow("Debug Image Window");
  }

  // Callback function for processing semantic color images.
  // This function applies thresholding and morphological operations
  // to extract regions with a specified intensity value (assumed to be yellow).
  void onSemImg(const sensor_msgs::ImageConstPtr& msg)
  {
      cv_bridge::CvImagePtr cv_ptr;
      try {
          // Convert ROS image message to OpenCV image (BGR8 encoding)
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      } catch (cv_bridge::Exception& e) {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
      }
      // Define threshold value to detect yellow objects
      int threshold_value = 215;
      cv::Mat grayImage;
      // Convert passed image to grayscale
      cv::cvtColor(cv_ptr->image, grayImage, cv::COLOR_BGR2GRAY);
      // Create a binary mask where pixels equal to the threshold value are set to 255
      object_mask_ = (grayImage == threshold_value);

      // Use morphological operations to remove noise from the mask
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
      cv::dilate(object_mask_, object_mask_, kernel);
      cv::morphologyEx(object_mask_, object_mask_, cv::MORPH_CLOSE, kernel);

      // Find contours in the binary mask to detect objects
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(object_mask_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      std::vector<std::vector<cv::Point>> filteredContours;
      for (const auto& contour : contours)
      {
          // Calculate contour area
          double area = cv::contourArea(contour);
          // Filter out small contours
          if (area >= 30) {
              filteredContours.push_back(contour);
              // Get the bounding box for the valid contour
              boundingBox = cv::boundingRect(contour);
              // Draw a rectangle around the detected object for debugging
              cv::rectangle(cv_ptr->image, boundingBox, cv::Scalar(0, 255, 0), 0.5);
          }
      }

      // If at least one contour is found, compute the centroid and update the flag.
      if (!filteredContours.empty())
      {
          is_light_found = true;
          // Compute the centroid of the bounding box. The y-coordinate is slightly adjusted.
          centroid.x = boundingBox.x + boundingBox.width / 2;
          centroid.y = boundingBox.y + boundingBox.height - 2;
          // Draw the centroid on the image for visualization
          cv::circle(cv_ptr->image, centroid, 1, cv::Scalar(0, 0, 255), -1);
      }
      else {
          is_light_found = false;
      }

      // Display the debug image in an OpenCV window.
      cv::imshow("Debug Image Window", cv_ptr->image);
      cv::waitKey(3);
  }

  // Callback function for processing depth images.
  // It uses the centroid determined from the color image to extract a depth value,
  // converts the 2D point (with depth) into a 3D camera coordinate,
  // and then transforms it to the world coordinate frame.
  void onDepthImg(const sensor_msgs::ImageConstPtr& depth_image)
  {
      try {
          // Convert ROS depth image message to OpenCV image (using 16UC1 encoding)
          cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
          if (is_light_found)
          {
              // Get the depth value at the computed centroid location
              centroid_depth = cv_ptr->image.at<uint16_t>(centroid.y, centroid.x);
              if (centroid_depth != 0)
              {
                  // Convert depth value from millimeters to meters
                  double depth_in_meter = centroid_depth / 1000.0;
                  // Form a homogeneous point in image space (x, y, depth)
                  cv::Mat_<double> homogeneousPoint(3, 1);
                  homogeneousPoint(0) = centroid.x * depth_in_meter;
                  homogeneousPoint(1) = centroid.y * depth_in_meter;
                  homogeneousPoint(2) = depth_in_meter;

                  // Convert 2D image coordinate with depth to 3D camera coordinate using the inverse of camera matrix
                  cv::Mat_<double> cameraCoordinates = camera_matrix.inv() * homogeneousPoint;

                  // Create a ROS PointStamped message defined in the camera's coordinate frame
                  geometry_msgs::PointStamped source_point;
                  source_point.header.frame_id = "Quadrotor/Sensors/DepthCamera";
                  // Use the timestamp provided by the camera_info callback
                  source_point.header.stamp = cur_time_stamp;
                  source_point.point.x = cameraCoordinates(0);
                  source_point.point.y = cameraCoordinates(1);
                  source_point.point.z = cameraCoordinates(2);

                  try {
                      // Check if a transform is available from the camera frame to the world frame
                      if (tfBuffer_.canTransform("world", source_point.header.frame_id, source_point.header.stamp, ros::Duration(1.0))) {
                          // Transform the point from the camera frame to the world frame
                          geometry_msgs::PointStamped transformed_point;
                          transformed_point = tfBuffer_.transform(source_point, "world");

                          // Log the transformed 3D point coordinates
                          ROS_WARN("Object of interest in: (%f, %f, %f)",
                                   transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);

                          // (Optional) Publish the transformed point
                          // point_pub_.publish(transformed_point);

                          // Create and publish a visualization marker for RViz.
                          // Each marker has a unique ID to prevent overwriting previous markers.
                          visualization_msgs::Marker marker;
                          marker.header.frame_id = "world";
                          marker.header.stamp = ros::Time::now();
                          marker.ns = "lantern_marker";
                          // Set a unique marker ID by incrementing marker_id_
                          marker.id = marker_id_++;
                          // Specify the marker type as a sphere.
                          marker.type = visualization_msgs::Marker::SPHERE;
                          // Marker action ADD indicates that the marker should be added
                          marker.action = visualization_msgs::Marker::ADD;
                          // Set marker position to the transformed 3D point
                          marker.pose.position = transformed_point.point;
                          // Set marker orientation to identity (no rotation)
                          marker.pose.orientation.x = 0.0;
                          marker.pose.orientation.y = 0.0;
                          marker.pose.orientation.z = 0.0;
                          marker.pose.orientation.w = 1.0;
                          // Set the scale of the marker (size in x, y, z directions)
                          marker.scale.x = 1;
                          marker.scale.y = 1;
                          marker.scale.z = 5;
                          // Set the marker color (red in this case, fully opaque)
                          marker.color.r = 1.0;
                          marker.color.g = 0.0;
                          marker.color.b = 0.0;
                          marker.color.a = 1.0;
                          // Set the lifetime of the marker (0 means it will not auto-delete)
                          marker.lifetime = ros::Duration(0);
                          // Publish the marker to be visualized in RViz
                          marker_pub_.publish(marker);
                      } else {
                          ROS_WARN("Transformation not available.");
                      }
                  } catch (tf2::TransformException& ex) {
                      // Log any transformation exceptions
                      ROS_ERROR("Failed to transform point: %s", ex.what());
                  }
              }
          }
      } catch (cv_bridge::Exception& e) {
          // Log exceptions from cv_bridge if there is a problem converting image messages
          ROS_ERROR("cv_bridge exception: %s", e.what());
      }
  }

  // Callback for camera info messages that contain the intrinsic parameters of the depth camera.
  // This callback updates the current timestamp and the camera intrinsic matrix.
  void onDepthInfo(const sensor_msgs::CameraInfo& depth_msg)
  {
      // Update the current time stamp using the camera info header
      cur_time_stamp = depth_msg.header.stamp;
      // Construct the camera intrinsic matrix from the incoming CameraInfo message
      camera_matrix = (cv::Mat_<double>(3, 3) << 
                       depth_msg.K[0], depth_msg.K[1], depth_msg.K[2],
                       depth_msg.K[3], depth_msg.K[4], depth_msg.K[5],
                       depth_msg.K[6], depth_msg.K[7], depth_msg.K[8]);
  }
};

int main(int argc, char** argv)
{
  // Initialize the ROS node with name "image_converter"
  ros::init(argc, argv, "image_converter");
  // Create an instance of the ImageConverter class
  ImageConverter ic;
  // Enter a loop, pumping callbacks
  ros::spin();
  return 0;
}


