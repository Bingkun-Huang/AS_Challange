#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/opencv.hpp>

class ImageConverter {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sem_img_sub_; // Subscriber for semantic RGB images
    image_transport::Subscriber depth_img_sub_; // Subscriber for depth images
    ros::Subscriber depth_info_sub_; // Subscriber for camera parameters
    ros::Publisher point_pub_; // Publisher for the detected object's 3D position
    
    cv::Mat object_mask_; // Mask to store detected object region
    cv::Mat camera_matrix; // Camera intrinsic matrix
    tf2_ros::Buffer tfBuffer; // Buffer to store transformations
    tf2_ros::TransformListener tfListener; // Transform listener
    cv::Point centroid; // Centroid of detected object in image coordinates
    double centroid_depth; // Depth value at the centroid position

public:
    ImageConverter() : it_(nh_), tfListener(tfBuffer) {
        // Subscribe to semantic camera image (RGB)
        sem_img_sub_ = it_.subscribe("/unity_ros/Quadrotor/Sensors/SemanticCamera/image_raw", 1, 
            &ImageConverter::onSemImg, this);
        
        // Subscribe to depth image
        depth_img_sub_ = it_.subscribe("/realsense/depth/image", 1, 
            &ImageConverter::onDepthImg, this);
        
        // Subscribe to camera intrinsic parameters
        depth_info_sub_ = nh_.subscribe("/realsense/depth/camera_info", 1, 
            &ImageConverter::onDepthInfo, this);
        
        // Publisher for the detected object's position in world coordinates
        point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("point/latern", 10);
    }

    // Callback function for semantic image processing
    void onSemImg(const sensor_msgs::ImageConstPtr& msg) {
        try {
            // Convert ROS image message to OpenCV format
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat grayImage;
            
            // Convert RGB image to grayscale
            cv::cvtColor(cv_ptr->image, grayImage, cv::COLOR_BGR2GRAY);
            
            // Thresholding to extract the object based on specific intensity value (assumed to be 215)
            int threshold_value = 215;
            object_mask_ = (grayImage == threshold_value);
            
            // Apply morphological operations to remove noise and enhance object detection
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
            cv::dilate(object_mask_, object_mask_, kernel);
            cv::morphologyEx(object_mask_, object_mask_, cv::MORPH_CLOSE, kernel);
            
            // Find contours in the binary image
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(object_mask_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            
            if (!contours.empty()) {
                // Find the largest contour and compute its bounding box
                auto largest_contour = *std::max_element(contours.begin(), contours.end(),
                    [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
                        return cv::contourArea(c1) < cv::contourArea(c2);
                    });
                cv::Rect boundingBox = cv::boundingRect(largest_contour);
                
                // Compute the centroid of the detected object
                centroid.x = boundingBox.x + boundingBox.width / 2;
                centroid.y = boundingBox.y + boundingBox.height - 2; // Slight offset to avoid edge artifacts
            }
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    // Callback function for depth image processing
    void onDepthImg(const sensor_msgs::ImageConstPtr& depth_image) {
        try {
            // Convert ROS depth image to OpenCV format
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
            centroid_depth = cv_ptr->image.at<uint16_t>(centroid.y, centroid.x);
            
            // Convert depth from mm to meters
            double depth_in_meter = centroid_depth / 1000.0;
            
            // Convert 2D image coordinates to 3D camera coordinates
            cv::Mat_<double> homogeneousPoint(3, 1);
            homogeneousPoint(0) = centroid.x * depth_in_meter;
            homogeneousPoint(1) = centroid.y * depth_in_meter;
            homogeneousPoint(2) = depth_in_meter;
            
            cv::Mat_<double> cameraCoordinates = camera_matrix.inv() * homogeneousPoint;
            
            // Transform the point from camera frame to world frame
            geometry_msgs::PointStamped source_point;
            source_point.header.frame_id = "Quadrotor/Sensors/DepthCamera";
            source_point.point.x = cameraCoordinates(0);
            source_point.point.y = cameraCoordinates(1);
            source_point.point.z = cameraCoordinates(2);
            
            geometry_msgs::PointStamped transformed_point;
            transformed_point = tfBuffer.transform(source_point, "world");
            
            // Publish the transformed world coordinate
            point_pub_.publish(transformed_point);
            ROS_WARN("Lantern detected at: (%f, %f, %f)",
                    transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        } catch (tf2::TransformException& ex) {
            ROS_WARN("TF2 Transform Exception: %s", ex.what());
        }
    }

    // Callback function to update camera intrinsic parameters
    void onDepthInfo(const sensor_msgs::CameraInfo& depth_msg) {
        camera_matrix = (cv::Mat_<double>(3, 3) << depth_msg.K[0], depth_msg.K[1], depth_msg.K[2],
                                                    depth_msg.K[3], depth_msg.K[4], depth_msg.K[5],
                                                    depth_msg.K[6], depth_msg.K[7], depth_msg.K[8]);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
