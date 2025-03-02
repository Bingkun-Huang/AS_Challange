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

// static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;

  ros::Subscriber depth_info_sub_;
  ros::Publisher point_pub_;

  image_transport::ImageTransport it_;

  image_transport::Subscriber sem_img_sub_;
  image_transport::Subscriber depth_img_sub_;
  

  image_transport::Publisher debug_image_pub_;


  cv::Mat object_mask_;
  cv::Point centroid;
  cv::Rect boundingBox;
  uint16_t centroid_depth;
  bool is_light_found;

  cv::Mat dist_coeffs;
  cv::Mat camera_matrix;

  ros::Time cur_time_stamp;


  public:
    ImageConverter()
      : it_(nh_)
    {
      // Subscrive to input video feed and publish output video feed
      sem_img_sub_ = it_.subscribe("/unity_ros/Quadrotor/Sensors/SemanticCamera/image_raw", 1,
        &ImageConverter::onSemImg, this);

      depth_img_sub_ = it_.subscribe("/realsense/depth/image", 1,
        &ImageConverter::onDepthImg, this);  

      depth_info_sub_ = nh_.subscribe("realsense/depth/camera_info", 1, 
      &ImageConverter::onDepthInfo, this);

      point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("point/latern", 10);

      // debug_image_pub_ = it_.advertise("/image_converter/debug_output_image", 1);

      cv::namedWindow("Debug Image Window", cv::WINDOW_NORMAL);
      cv::resizeWindow("Debug Image Window", 640, 480);
    }


    ~ImageConverter()
    {
      cv::destroyWindow("Debug Image Window");
    }

    void onSemImg(const sensor_msgs::ImageConstPtr& msg)
    {
      cv_bridge::CvImagePtr cv_ptr;
      cv_bridge::CvImagePtr cv_mask_ptr;

      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
      // set threshold for the yellow color
      int threshold_value = 215;

      // convert the color image to gray image
      cv::Mat grayImage;
      cv::cvtColor(cv_ptr->image, grayImage, cv::COLOR_BGR2GRAY);

      // Using the threshold to get the pixels
      object_mask_ = (grayImage == threshold_value);

      // Using morphological operation to merge the traget
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
      cv::dilate(object_mask_, object_mask_, kernel);
      cv::morphologyEx(object_mask_, object_mask_, cv::MORPH_CLOSE, kernel);

      // Get the contours
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(object_mask_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


        
      std::vector<std::vector<cv::Point>> filteredContours;
      for (const auto& contour : contours) 
      {
        double area = cv::contourArea(contour);
        if (area >= 30) {
            filteredContours.push_back(contour);
            boundingBox = cv::boundingRect(contour);
            cv::rectangle(cv_ptr->image, boundingBox, cv::Scalar(0, 255, 0), 0.5);
        }
      }

      int num_of_contours = filteredContours.size();
      
      if (num_of_contours > 0)
      {

        is_light_found = true;

        // Calculate the centroid of the contour
        centroid.x = boundingBox.x + boundingBox.width / 2;
        centroid.y = boundingBox.y + boundingBox.height - 2;

        // Draw the contour and centroid
        cv::circle(cv_ptr->image, centroid, 1, cv::Scalar(0, 0, 255), -1);

      } else {
        is_light_found = false;
        // ROS_INFO("Searching...");
      }

      // Update GUI Window
      cv::imshow("Debug Image Window", cv_ptr->image);
      cv::waitKey(3);
    }

    void onDepthImg(const sensor_msgs::ImageConstPtr& depth_image)
    {
      
      try {
 
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);

        int image_width = cv_ptr->image.cols;
        int image_height = cv_ptr->image.rows;

        if (is_light_found)
        {
          centroid_depth = cv_ptr->image.at<uint16_t>(centroid.y, centroid.x);

          // Show the image only for debug
          cv::Mat debug_resultImage;

          if (centroid_depth != 0)
          {
            // ROS_INFO("Latern found with depth value at (%d, %d): %u", centroid.x, centroid.y, centroid_depth);

            double depth_in_meter = centroid_depth / 1000.0;

            cv::Mat_<double> homogeneousPoint(3, 1);

            homogeneousPoint(0) = centroid.x * depth_in_meter;
            homogeneousPoint(1) = centroid.y * depth_in_meter;
            homogeneousPoint(2) = depth_in_meter;

            cv::Mat_<double> cameraCoordinates = camera_matrix.inv() * homogeneousPoint;

            // // // 创建tfBuffer和tfListener
            tf2_ros::Buffer tfBuffer(ros::Duration(3000));
            tf2_ros::TransformListener tfListener(tfBuffer);

            // Reference: https://github.com/ros-planning/navigation/blob/noetic-devel/amcl/src/amcl_node.cpp
            ros::Duration transform_tolerance;
            transform_tolerance.fromSec(2);
            
            // // // Create a point in a source frame
            geometry_msgs::PointStamped source_point;
            source_point.header.frame_id = "Quadrotor/Sensors/DepthCamera";
            source_point.header.stamp = cur_time_stamp + transform_tolerance;
            source_point.point.x = cameraCoordinates(0);
            source_point.point.y = cameraCoordinates(1);
            source_point.point.z = cameraCoordinates(2);

            ros::Duration(1.0).sleep();

            try {
                if (tfBuffer.canTransform("world", source_point.header.frame_id, 
                                source_point.header.stamp, ros::Duration(1.0))) {


                    geometry_msgs::PointStamped transformed_point;
                    transformed_point = tfBuffer.transform(source_point, "world");


                    ROS_WARN("Latern in: (%f, %f, %f)",
                transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);

                // point_pub_.publish(transformed_point);
                } else {

                    ROS_WARN("Transformation not available.");
                }
            } catch (tf2::TransformException& ex) {
                ROS_ERROR("Failed to transform point: %s", ex.what());
            }
          }
        }
      } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
      }
    }

    void onDepthInfo(const sensor_msgs::CameraInfo& depth_msg)
    {
      // dist_coeffs = cv::Mat_<double>(1, 5) << *(depth_msg.D.data());

      cur_time_stamp = depth_msg.header.stamp;

      camera_matrix = (cv::Mat_<double>(3, 3) << depth_msg.K[0], depth_msg.K[1], depth_msg.K[2],
                                                depth_msg.K[3], depth_msg.K[4], depth_msg.K[5],
                                                depth_msg.K[6], depth_msg.K[7], depth_msg.K[8]);                                      
    }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}