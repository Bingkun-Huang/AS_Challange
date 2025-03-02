#include <ros/ros.h>
#include <image_transport/image_transport.h>            // For transporting images in ROS
#include <cv_bridge/cv_bridge.h>                        // To convert between ROS image messages and OpenCV images
#include <sensor_msgs/image_encodings.h>                // To handle image encoding types
#include <sensor_msgs/Image.h>                          // ROS image message type
#include <sensor_msgs/CameraInfo.h>                     // ROS camera info message type
#include <geometry_msgs/PointStamped.h>                 // ROS message type for stamped points (with header and frame info)
#include <tf2_ros/transform_listener.h>                 // TF2 listener to listen for coordinate transforms
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>         // Utilities for transforming geometry messages via TF2
#include <opencv2/opencv.hpp>                           // OpenCV library for image processing

// The ImageConverter class handles the processing of a semantic (RGB) image and a depth image,
// then converts the detected object's position from image coordinates to 3D world coordinates.
class ImageConverter {
private:
    // ROS node handle for creating publishers, subscribers, etc.
    ros::NodeHandle nh_;

    // ImageTransport object allows subscribing and publishing of images in ROS
    image_transport::ImageTransport it_;

    // Subscribers for the semantic (RGB) image and the depth image
    image_transport::Subscriber sem_img_sub_; // Subscriber for semantic RGB images
    image_transport::Subscriber depth_img_sub_; // Subscriber for depth images

    // Subscriber for camera information (intrinsics)
    ros::Subscriber depth_info_sub_;

    // Publisher for the detected object's position (as a PointStamped message)
    ros::Publisher point_pub_;

    // OpenCV Mat object to store the binary mask of the detected object
    cv::Mat object_mask_;

    // Camera intrinsic matrix (3x3) obtained from camera info message
    cv::Mat camera_matrix;

    // TF2 buffer and listener to receive coordinate frame transformations
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    // Detected object's centroid in image coordinates (pixel coordinates)
    cv::Point centroid;

    // Depth value at the centroid position (converted to meters later)
    double centroid_depth;

public:
    // Constructor initializes the image transport, TF listener, and sets up subscriptions and publishers.
    ImageConverter() 
        : it_(nh_), tfListener(tfBuffer) // Initialize image transport and TF listener with the buffer
    {
        // Subscribe to the semantic camera image topic (assumed to be published as BGR8)
        sem_img_sub_ = it_.subscribe("/unity_ros/Quadrotor/Sensors/SemanticCamera/image_raw", 1, 
            &ImageConverter::onSemImg, this);

        // Subscribe to the depth image topic (assumed to be 16-bit unsigned integer, representing depth in mm)
        depth_img_sub_ = it_.subscribe("/realsense/depth/image", 1, 
            &ImageConverter::onDepthImg, this);

        // Subscribe to the camera information topic to get intrinsic parameters
        depth_info_sub_ = nh_.subscribe("/realsense/depth/camera_info", 1, 
            &ImageConverter::onDepthInfo, this);

        // Advertise a publisher for the detected object's 3D position (in world coordinates)
        point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("point/latern", 10);
    }

    // Callback function for processing the semantic image.
    // This function converts the input ROS image to an OpenCV image, converts it to grayscale,
    // applies thresholding and morphological operations to create a binary mask, and then finds
    // contours to locate the object of interest. The centroid of the largest contour is computed.
    void onSemImg(const sensor_msgs::ImageConstPtr& msg) {
        try {
            // Convert the ROS image message to an OpenCV image (BGR8 format)
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat grayImage;

            // Convert the color image to grayscale for easier thresholding
            cv::cvtColor(cv_ptr->image, grayImage, cv::COLOR_BGR2GRAY);

            // Use a fixed threshold value to extract pixels that match a specific intensity (assumed to be 215)
            int threshold_value = 215;
            object_mask_ = (grayImage == threshold_value);

            // Apply morphological operations to reduce noise and connect nearby regions:
            // 1. Dilation enlarges bright regions.
            // 2. Closing operation helps fill gaps.
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
            cv::dilate(object_mask_, object_mask_, kernel);
            cv::morphologyEx(object_mask_, object_mask_, cv::MORPH_CLOSE, kernel);

            // Find contours in the binary mask
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(object_mask_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            // If any contours were found, select the largest contour and compute its bounding box
            if (!contours.empty()) {
                // Find the largest contour based on area
                auto largest_contour = *std::max_element(contours.begin(), contours.end(),
                    [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
                        return cv::contourArea(c1) < cv::contourArea(c2);
                    });
                cv::Rect boundingBox = cv::boundingRect(largest_contour);

                // Compute the centroid of the bounding box; subtracting 2 from y to offset the edge if necessary
                centroid.x = boundingBox.x + boundingBox.width / 2;
                centroid.y = boundingBox.y + boundingBox.height - 2;
            }
        } catch (cv_bridge::Exception& e) {
            // If conversion fails, log the error message
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    // Callback function for processing the depth image.
    // It uses the previously computed centroid from the semantic image, extracts the depth value at that pixel,
    // converts the 2D pixel location and depth value into a 3D point in the camera coordinate frame,
    // and then transforms it into the world coordinate frame using TF2.
    void onDepthImg(const sensor_msgs::ImageConstPtr& depth_image) {
        try {
            // Convert the ROS depth image message to an OpenCV image (TYPE_16UC1 encoding: 16-bit unsigned, 1 channel)
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);

            // Extract the depth value at the centroid; this value is in millimeters
            centroid_depth = cv_ptr->image.at<uint16_t>(centroid.y, centroid.x);

            // Convert the depth from millimeters to meters for 3D reconstruction
            double depth_in_meter = centroid_depth / 1000.0;

            // Create a homogeneous coordinate vector from the 2D image coordinates and depth value
            cv::Mat_<double> homogeneousPoint(3, 1);
            homogeneousPoint(0) = centroid.x * depth_in_meter;
            homogeneousPoint(1) = centroid.y * depth_in_meter;
            homogeneousPoint(2) = depth_in_meter;

            // Use the inverse of the camera intrinsic matrix to convert to 3D camera coordinates
            cv::Mat_<double> cameraCoordinates = camera_matrix.inv() * homogeneousPoint;

            // Create a PointStamped message for the 3D point in the camera frame
            geometry_msgs::PointStamped source_point;
            source_point.header.frame_id = "Quadrotor/Sensors/DepthCamera";
            source_point.point.x = cameraCoordinates(0);
            source_point.point.y = cameraCoordinates(1);
            source_point.point.z = cameraCoordinates(2);

            // Transform the point from the camera frame to the world frame using TF2
            geometry_msgs::PointStamped transformed_point;
            transformed_point = tfBuffer.transform(source_point, "world");

            // Publish the transformed point (in world coordinates)
            point_pub_.publish(transformed_point);

            // Log the detected 3D position to the console
            ROS_WARN("Lantern detected at: (%f, %f, %f)",
                     transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);
        } catch (cv_bridge::Exception& e) {
            // Log any exceptions from cv_bridge conversion
            ROS_ERROR("cv_bridge exception: %s", e.what());
        } catch (tf2::TransformException& ex) {
            // Log any exceptions that occur during TF2 transformation
            ROS_WARN("TF2 Transform Exception: %s", ex.what());
        }
    }

    // Callback function to update the camera intrinsic matrix using the camera info message.
    // This matrix is used later to convert image coordinates into 3D camera coordinates.
    void onDepthInfo(const sensor_msgs::CameraInfo& depth_msg) {
        // Construct the camera intrinsic matrix from the K parameter in the CameraInfo message
        camera_matrix = (cv::Mat_<double>(3, 3) << depth_msg.K[0], depth_msg.K[1], depth_msg.K[2],
                                                    depth_msg.K[3], depth_msg.K[4], depth_msg.K[5],
                                                    depth_msg.K[6], depth_msg.K[7], depth_msg.K[8]);
    }
};

int main(int argc, char** argv) {
    // Initialize the ROS node with the name "image_converter"
    ros::init(argc, argv, "image_converter");

    // Create an instance of the ImageConverter class which sets up subscriptions and publishers
    ImageConverter ic;

    // Keep the node running and processing callbacks until shutdown
    ros::spin();

    return 0;
}
