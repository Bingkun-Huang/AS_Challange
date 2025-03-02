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

class FrontierDetecter 
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

   
    image_transport::Subscriber depth_img_sub_;
    ros::Subscriber depth_info_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber state_waypoint_sub_;
    ros::Subscriber length_his_queue_sub_;
    ros::Publisher point_pub_;
    ros::Publisher state_outside_cave_pub_;

    // Variables
    cv::Mat camera_matrix;
    cv::Mat depth_img;
    ros::Time cur_time_stamp;
    double cur_x, cur_y, cur_z;
    int64_t length_his_queue = 0;

    bool isWaypointsEmpty = true;
    bool maybeOutsideCave = false;
    bool hasPubLastPoint = false;

    // -------------------------------------------
    // Callback 
    // -------------------------------------------
    void onDepthImg(const sensor_msgs::ImageConstPtr& depth_image) 
    {
        try 
        {
            depth_img = cv_bridge::toCvShare(depth_image, sensor_msgs::image_encodings::TYPE_16UC1)->image;
        } catch (cv_bridge::Exception& e) 
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void onDepthInfo(const sensor_msgs::CameraInfo& msg) 
    {
        cur_time_stamp = msg.header.stamp;
        camera_matrix = (cv::Mat_<double>(3, 3) << msg.K[0], msg.K[1], msg.K[2],
                                                   msg.K[3], msg.K[4], msg.K[5],
                                                   msg.K[6], msg.K[7], msg.K[8]);
        camera_matrix.convertTo(camera_matrix, CV_64F);
    }

    void onOdom(const nav_msgs::Odometry::ConstPtr& msg) 
    {
        cur_x = msg->pose.pose.position.x;
        cur_y = msg->pose.pose.position.y;
        cur_z = msg->pose.pose.position.z;
    }

    void onLengthHisQueue(const std_msgs::Int64::ConstPtr& msg) 
    {
        if (msg->data > 20000) 
        {
            length_his_queue = msg->data;
        }
    }

    void onStateWaypoints(const std_msgs::Bool::ConstPtr& msg) 
    {
        if (msg->data) {
            isWaypointsEmpty = true;
            calcCenterCorrection();
        } else 
        {
            isWaypointsEmpty = false;
            ROS_WARN("Queue is not empty");
        }
    }

    // -------------------------------------------
    // Logic - detect the free space 
    // -------------------------------------------
    void calcCenterCorrection() 
    {
        if (depth_img.empty()) return;

        // Step 1: 深度图二值化+找连通域
        cv::Mat normalized, segmented;
        depth_img.convertTo(normalized, CV_8UC1, 255.0 / 65535.0);
        cv::threshold(normalized, segmented, 0, 255, cv::THRESH_BINARY_INV);

        cv::Mat labels, stats, centroids;
        int nLabels = cv::connectedComponentsWithStats(segmented, labels, stats, centroids);

        if (nLabels <= 1) 
        {
            publishFallbackPoint();
            return;
        }

        ROS_ERROR("Labels = %d", nLabels);

// Step 2: 找最大和第二大连通域
// Step 2: 找最大和第二大连通域（过滤面积<500的连通域）
        int largestIndex = -1, secondLargestIndex = -1;
        int maxArea = 0, secondMaxArea = 0;
        const int AREA_THRESHOLD = 500;

        for (int i = 1; i < nLabels; i++) 
        {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);

            if (area < AREA_THRESHOLD) 
            {
             // 小于500的直接跳过
                continue;
            }

            if (area > maxArea) 
        {
            secondMaxArea = maxArea;
            secondLargestIndex = largestIndex;
            maxArea = area;
            largestIndex = i;
         } 
        else if (area > secondMaxArea) 
        {
        secondMaxArea = area;
        secondLargestIndex = i;
     }
}

// 如果最大和第二大都不存在（说明所有连通域都太小），直接放弃：
        if (largestIndex == -1) 
        {
        ROS_WARN("No valid connected component with area > %d found, publishing fallback point.", AREA_THRESHOLD);
        publishFallbackPoint();
        return;
        }

        ROS_ERROR("MaxArea = %d (Label = %d)", maxArea, largestIndex);
        ROS_ERROR("SecondMaxArea = %d (Label = %d)", secondMaxArea, secondLargestIndex);


        if (maxArea > 20000) maybeOutsideCave = true;

        // Step 3: 找连通域的中心
        cv::Mat mask = (labels == largestIndex);
        cv::Mat edges;
        cv::Canny(mask, edges, 50, 200);
        cv::Moments m = cv::moments(edges, true);
        cv::Point center(m.m10 / m.m00, m.m01 / m.m00);

        // Step 4: 像素点->相机3D点
        double depth_in_meter = 36.0; //36
        cv::Mat_<double> camPoint = camera_matrix.inv() * (cv::Mat_<double>(3,1) << center.x * depth_in_meter, center.y * depth_in_meter, depth_in_meter);

        // Step 5: 相机系->world系
        geometry_msgs::PointStamped camStamped, worldStamped;
        camStamped.header.frame_id = "Quadrotor/Sensors/DepthCamera";
        camStamped.header.stamp = cur_time_stamp;
        camStamped.point.x = camPoint(0);
        camStamped.point.y = camPoint(1);
        camStamped.point.z = camPoint(2);

        transformToWorld(camStamped, worldStamped);

        // Step 6: 发布目标点
        if (maybeOutsideCave && length_his_queue > 500 && !hasPubLastPoint) 
        {
            publishFinalPoint();
        } else {
            point_pub_.publish(worldStamped);
            ROS_INFO("Publishing NEXT POINT based on deepth camera: (%.2f, %.2f, %.2f)", 
             worldStamped.point.x, 
             worldStamped.point.y, 
             worldStamped.point.z);
        }
    }

    void publishFallbackPoint() {
        geometry_msgs::PointStamped point;
        point.header.frame_id = "world";
        point.point.x = cur_x + 0.1;
        point.point.y = cur_y + 0.1;
        point.point.z = cur_z - 0.1;
        ROS_WARN("No valid cave found. Publishing fallback point.");
        point_pub_.publish(point);
    }

    void publishFinalPoint() {
        geometry_msgs::PointStamped point;
        point.header.frame_id = "world";
        point.point.x = -59.0;
        point.point.y = 0.84;
        point.point.z = 12.0;
        ROS_WARN("Reached outside of cave. Publishing final point.");
        point_pub_.publish(point);
        hasPubLastPoint = true;
    }

    void transformToWorld(const geometry_msgs::PointStamped& input, geometry_msgs::PointStamped& output) {
        static tf2_ros::Buffer tfBuffer;
        static tf2_ros::TransformListener tfListener(tfBuffer);

        try {
            tfBuffer.canTransform("world", input.header.frame_id, input.header.stamp, ros::Duration(1.0));
            output = tfBuffer.transform(input, "world");
        } catch (tf2::TransformException &ex) {
            ROS_WARN("TF failed: %s", ex.what());
        }
    }

public:
    FrontierDetecter() : it_(nh_) {
        depth_img_sub_ = it_.subscribe("/realsense/depth/image", 1, &FrontierDetecter::onDepthImg, this);
        depth_info_sub_ = nh_.subscribe("realsense/depth/camera_info", 1, &FrontierDetecter::onDepthInfo, this);
        odom_sub_ = nh_.subscribe("/current_state_est", 1, &FrontierDetecter::onOdom, this);
        state_waypoint_sub_ = nh_.subscribe("/state/waypoints_queue", 1, &FrontierDetecter::onStateWaypoints, this);
        length_his_queue_sub_ = nh_.subscribe("/length_his_queue", 1, &FrontierDetecter::onLengthHisQueue, this);
        point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("point/correction", 1);
        state_outside_cave_pub_ = nh_.advertise<std_msgs::Bool>("/state/is_outside_cave", 1);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "Frontier_Detecter");
    FrontierDetecter corrector;
    ROS_INFO("Frontier_Detecter Started");
    ros::spin();
    return 0;
}
