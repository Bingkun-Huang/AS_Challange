#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int64.h"
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <fstream>

class FrontierDetecter 
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber depth_img_sub_;
    ros::Subscriber depth_info_sub_, odom_sub_, state_waypoint_sub_, length_his_queue_sub_;
    ros::Publisher point_pub_, state_outside_cave_pub_;
    
    cv::Mat camera_matrix, depth_img;
    ros::Time cur_time_stamp;
    double cur_x = 0, cur_y = 0, cur_z = 0;
    int64_t length_his_queue = 0;
    
    bool isWaypointsEmpty = true, maybeOutsideCave = false, hasPubLastPoint = false;
    
    std::vector<Eigen::Vector3d> reverse_waypoints_;
    int largestIndex = -1, secondLargestIndex = -1;
    bool exploringSecondArea = false;

public:
    FrontierDetecter() : it_(nh_) {
        depth_img_sub_ = it_.subscribe("/realsense/depth/image", 1, &FrontierDetecter::onDepthImg, this);
        depth_info_sub_ = nh_.subscribe("/realsense/depth/camera_info", 1, &FrontierDetecter::onDepthInfo, this);
        odom_sub_ = nh_.subscribe("/current_state_est", 1, &FrontierDetecter::onOdom, this);
        state_waypoint_sub_ = nh_.subscribe("/state/waypoints_queue", 1, &FrontierDetecter::onStateWaypoints, this);
        length_his_queue_sub_ = nh_.subscribe("/length_his_queue", 1, &FrontierDetecter::onLengthHisQueue, this);
        point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("point/correction", 1);
    }

    void calcCenterCorrection() {
        if (depth_img.empty()) return;

        cv::Mat normalized, segmented;
        depth_img.convertTo(normalized, CV_8UC1, 255.0 / 65535.0);
        cv::threshold(normalized, segmented, 0, 255, cv::THRESH_BINARY_INV);

        cv::Mat labels, stats, centroids;
        int nLabels = cv::connectedComponentsWithStats(segmented, labels, stats, centroids);
        if (nLabels <= 1) { publishFallbackPoint(); return; }

        findLargestTwoAreas(nLabels, stats);

        if (largestIndex == -1) {
            publishFallbackPoint();
            return;
        }

        if (secondLargestIndex != -1) {
            publishAreaCenter(secondLargestIndex);
            exploringSecondArea = true;
            reverse_waypoints_.clear();
        } else {
            publishAreaCenter(largestIndex);
            exploringSecondArea = false;
        }
    }

    void findLargestTwoAreas(int nLabels, const cv::Mat& stats) {
        const int AREA_THRESHOLD = 500;
        int maxArea = 0, secondMaxArea = 0;
        largestIndex = secondLargestIndex = -1;

        for (int i = 1; i < nLabels; i++) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area < AREA_THRESHOLD) continue;

            if (area > maxArea) {
                secondMaxArea = maxArea;
                secondLargestIndex = largestIndex;
                maxArea = area;
                largestIndex = i;
            } else if (area > secondMaxArea) {
                secondMaxArea = area;
                secondLargestIndex = i;
            }
        }

        ROS_INFO("Largest Area: %d (Label=%d)", maxArea, largestIndex);
        ROS_INFO("Second Largest Area: %d (Label=%d)", secondMaxArea, secondLargestIndex);
    }

    void publishAreaCenter(int index) {
        cv::Point center = calcRegionCenter(index);
        Eigen::Vector3d targetPoint = pixelTo3D(center);
        publishPoint(targetPoint);
        reverse_waypoints_.push_back(Eigen::Vector3d(cur_x, cur_y, cur_z));
    }

    cv::Point calcRegionCenter(int labelIndex) {
        cv::Mat mask = (depth_img == labelIndex);
        cv::Mat edges;
        cv::Canny(mask, edges, 50, 200);
        cv::Moments m = cv::moments(edges, true);
        return cv::Point(m.m10 / m.m00, m.m01 / m.m00);
    }

    Eigen::Vector3d pixelTo3D(const cv::Point& center) {
        double depth = 20.0;
        cv::Mat_<double> camPoint = camera_matrix.inv() * (cv::Mat_<double>(3,1) << center.x*depth, center.y*depth, depth);
        return {camPoint(0), camPoint(1), camPoint(2)};
    }

    void publishPoint(const Eigen::Vector3d& point) {
        geometry_msgs::PointStamped worldPoint;
        camToWorld(point, worldPoint);
        point_pub_.publish(worldPoint);

        ROS_INFO("Publishing Target Point: [%.2f, %.2f, %.2f]", 
                 worldPoint.point.x, worldPoint.point.y, worldPoint.point.z);
    }

    void camToWorld(const Eigen::Vector3d& camPoint, geometry_msgs::PointStamped& worldPoint) {
        geometry_msgs::PointStamped camStamped;
        camStamped.header.frame_id = "Quadrotor/Sensors/DepthCamera";
        camStamped.header.stamp = cur_time_stamp;
        camStamped.point.x = camPoint.x();
        camStamped.point.y = camPoint.y();
        camStamped.point.z = camPoint.z();
        transformToWorld(camStamped, worldPoint);
    }

    void onDepthImg(const sensor_msgs::ImageConstPtr& depth_image) {
        depth_img = cv_bridge::toCvShare(depth_image, sensor_msgs::image_encodings::TYPE_16UC1)->image;
    }

    void onDepthInfo(const sensor_msgs::CameraInfo& msg) {
        cur_time_stamp = msg.header.stamp;
        camera_matrix = (cv::Mat_<double>(3,3) << msg.K[0],0,msg.K[2], 0,msg.K[4],msg.K[5], 0,0,1);
    }

    void onOdom(const nav_msgs::Odometry::ConstPtr& msg) {
        cur_x = msg->pose.pose.position.x;
        cur_y = msg->pose.pose.position.y;
        cur_z = msg->pose.pose.position.z;
    }

    void onStateWaypoints(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data) calcCenterCorrection();
    }

    void onLengthHisQueue(const std_msgs::Int64::ConstPtr& msg) {
        if (msg->data > 20000) {
            length_his_queue = msg->data;
        }
    }

    void transformToWorld(const geometry_msgs::PointStamped& input, geometry_msgs::PointStamped& output) {
        static tf2_ros::Buffer tfBuffer;
        static tf2_ros::TransformListener tfListener(tfBuffer);
        tfBuffer.transform(input, output, "world");
    }

    void publishFallbackPoint() {
        geometry_msgs::PointStamped point;
        point.header.frame_id = "world";
        point.point.x = cur_x + 0.1;
        point.point.y = cur_y + 0.1;
        point.point.z = cur_z - 0.1;
        point_pub_.publish(point);
        ROS_WARN("Publishing fallback point: [%.2f, %.2f, %.2f]", 
                 point.point.x, point.point.y, point.point.z);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "FrontierDetecter");
    FrontierDetecter detector;
    ros::spin();
}
