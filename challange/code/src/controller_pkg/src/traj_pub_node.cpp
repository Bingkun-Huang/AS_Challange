#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sstream>
#include <iostream>
#include <math.h>

#define PI M_PI

class TrajectoryPublisher
{
public:
    TrajectoryPublisher() : linear_scale_(0.1), angular_scale_(0.1)
    {
        desired_state_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("desired_state", 1);
        key_sub_ = nh_.subscribe("keyboard_input", 1, &TrajectoryPublisher::keyboardCallback, this);
        current_pose_sub_ = nh_.subscribe("current_state_est", 1, &TrajectoryPublisher::currentPoseCallback, this);
    }

    void run()
    {
        ros::Rate loop_rate(500);
        ros::Time start(ros::Time::now());

        while (ros::ok())
        {
            updateDesiredPose();
            publishDesiredState();
            broadcastTransform();

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher desired_state_pub_;
    ros::Subscriber key_sub_;
    ros::Subscriber current_pose_sub_;
    tf::TransformBroadcaster tf_broadcaster_;
    tf::Transform desired_pose_;
    nav_msgs::Odometry current_pose_;
    double linear_scale_;
    double angular_scale_;

    void keyboardCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        updateDesiredPoseFromKeyboard(msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.z);
    }

    void currentPoseCallback(const nav_msgs::Odometry& cur_state)
    {
        current_pose_ = cur_state;
    }

    void updateDesiredPoseFromKeyboard(double forward, double right, double up, double yaw)
    {
        tf::Vector3 displacement(right * linear_scale_, forward * linear_scale_, up * linear_scale_);
        desired_pose_.setOrigin(desired_pose_.getOrigin() + displacement);

        tf::Quaternion q;
        q.setRPY(0, 0, desired_pose_.getRotation().getAngle() + yaw * angular_scale_);
        desired_pose_.setRotation(q);
    }

    void updateDesiredPose()
    {
        // 结合键盘输入和当前位置进行计算
        // 使用当前位置作为目标位置的初始位置，然后添加键盘输入的偏移
        tf::Vector3 current_position(current_pose_.pose.pose.position.x, current_pose_.pose.pose.position.y, current_pose_.pose.pose.position.z);
        desired_pose_.setOrigin(current_position);

        // 添加键盘输入的偏移
        updateDesiredPoseFromKeyboard(0.0, 0.0, 0.0, 0.0);
    }

    void publishDesiredState()
    {
        trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
        msg.transforms.resize(1);
        msg.transforms[0].translation.x = desired_pose_.getOrigin().x();
        msg.transforms[0].translation.y = desired_pose_.getOrigin().y();
        msg.transforms[0].translation.z = desired_pose_.getOrigin().z();
        msg.transforms[0].rotation.x = desired_pose_.getRotation().getX();
        msg.transforms[0].rotation.y = desired_pose_.getRotation().getY();
        msg.transforms[0].rotation.z = desired_pose_.getRotation().getZ();
        msg.transforms[0].rotation.w = desired_pose_.getRotation().getW();

        desired_state_pub_.publish(msg);
    }

    void broadcastTransform()
    {
        tf_broadcaster_.sendTransform(tf::StampedTransform(desired_pose_, ros::Time::now(), "world", "av-desired"));
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_publisher");
    TrajectoryPublisher traj_publisher;
    traj_publisher.run();
    return 0;
}
