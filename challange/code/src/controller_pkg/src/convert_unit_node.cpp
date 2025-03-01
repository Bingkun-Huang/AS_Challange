#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class DepthUnitConverter {
public:
    DepthUnitConverter()
        : it_(nh_) {
        // 订阅深度图像
        depth_sub_ = it_.subscribe("/realsense/depth/image", 1, &DepthUnitConverter::depthCallback, this);

        // 创建一个发布者发布转换后的深度图像
        depth_pub_ = it_.advertise("/realsense/depth/image_in_m", 1);
    }

    void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImageConstPtr cv_ptr;
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);

            // 创建一个新的深度图像
            cv::Mat converted_img;
            cv_ptr->image.convertTo(converted_img, CV_32FC1, 1.0 / 1000.0); // 从毫米转换为米

            // 将转换后的深度图像再次转换为ROS消息
            cv_bridge::CvImage out_msg;
            out_msg.header = cv_ptr->header; // 使用相同的消息头
            out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // 更新编码类型
            out_msg.image = converted_img; // 设置转换后的图像

            // 发布转换后的深度图像
            depth_pub_.publish(out_msg.toImageMsg());
        } catch (const cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber depth_sub_;
    image_transport::Publisher depth_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "depth_unit_converter");
    DepthUnitConverter depth_converter;
    ros::spin();
    return 0;
}