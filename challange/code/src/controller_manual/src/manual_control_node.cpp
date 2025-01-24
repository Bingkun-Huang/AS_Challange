#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

#include <nav_msgs/Odometry.h>
#define PI M_PI
#include <math.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <eigen3/Eigen/Dense>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>

// #include <mav_msgs/Actuators.h>

class Manual_controlNode{
  ros::NodeHandle nh;
  //  PART 1 |  Declare ROS callback handlers
  //   1. one subscriber (from unity_state)
  //   2. one subscriber (from keyboard_get)
  //   3. one publisher (for controller_node to generate propeller speeds)

  ros::Subscriber current_state;
  ros::Subscriber keyboard_state;
  ros::Publisher desired_state;
  ros::Timer timer;

  Eigen::Matrix3d J;     // Inertia Matrix
  Eigen::Vector3d e3;    // [0,0,1]

  // Controller internals
  // Current state
  Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
  Eigen::Matrix3d R;     // current orientation of the UAV
  Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame
  double yaw;            // current yaw angle

  // Desired state
  Eigen::Vector3d xd;    // desired position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d vd;    // desired velocity of the UAV's c.o.m. in the world frame
  Eigen::Vector3d ad;    // desired acceleration of the UAV's c.o.m. in the world frame
  double yawd;           // desired yaw angle

  Eigen::Vector3d linear_increment; // x,y,z increment
  double angular_increment; 

  double hz;             // frequency of the main control loop

public:
  Manual_controlNode():e3(0,0,1),hz(100.0),xd(0, 0, 0){

      //  PART 2 |  Initialize ROS callback handlers
      // In this section, need to initialize handlers from part 1.
      //  - bind manual_controlNode::onCurrentState() to the topic "current_state"
      //    given by the "hz" variable
      
      current_state = nh.subscribe("current_state", 10, &Manual_controlNode::onCurrentState, this);

      keyboard_state = nh.subscribe("keyboard_input", 10, &Manual_controlNode::onKeyboardInput, this);

      // desired_state = nh.advertise<nav_msgs::Odometry>("desired_state", 10);
      desired_state = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("desired_state", 10);

      timer = nh.createTimer(ros::Duration(1.0 / hz), &Manual_controlNode::timerCallback, this);

      yawd = 0.0;
      // yawd = PI/2; // why here PI/2 ##########################################
      J << 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0;
      linear_increment = Eigen::Vector3d::Zero();
      angular_increment = 0.0;
  }

  void onCurrentState(const nav_msgs::Odometry& cur_state){
      //  Objective: fill in x, v, R and omega

      // Get the current position and velocity from the incoming ROS message and
      // fill in the class member variables x, v, R and omega accordingly.
      //
      //  CAVEAT: cur_state.twist.twist.angular is in the world frame, while omega
      //          needs to be in the body frame!
      
      x << cur_state.pose.pose.position.x,
           cur_state.pose.pose.position.y,
           cur_state.pose.pose.position.z;

      v << cur_state.twist.twist.linear.x,
           cur_state.twist.twist.linear.y,
           cur_state.twist.twist.linear.z;

      omega << cur_state.twist.twist.angular.x,
               cur_state.twist.twist.angular.y,
               cur_state.twist.twist.angular.z;

      Eigen::Quaterniond q;

      // tf::quaternionMsgToEigen (cur_state.pose.pose.orientation, q);
      const geometry_msgs::Quaternion& orientation = cur_state.pose.pose.orientation;
      q = Eigen::Quaterniond(orientation.w, orientation.x, orientation.y, orientation.z);

      R = q.toRotationMatrix();
      omega = R.transpose()*omega; // Rotate omega

      yaw = atan2(R(1, 0), R(0, 0));
      yawd = yaw;
      xd = x;
  }

  void onKeyboardInput(const std_msgs::UInt8& msg)
    {
      // increment reset
      linear_increment.setZero();
      angular_increment = 0.0;

      switch (msg.data)
      {
        case 'w': // translantion forward
            linear_increment = Eigen::Vector3d(0.1, 0.0, 0.0);
            break;
        case 's': // translantion backward
            linear_increment = Eigen::Vector3d(-0.1, 0.0, 0.0);
            break;
        case 'a': // translantion left
            linear_increment = Eigen::Vector3d(0.0, 0.1, 0.0);
            break;
        case 'd': // translantion right
            linear_increment = Eigen::Vector3d(0.0, -0.1, 0.0);
            break;
        case 'q': // translation up
            linear_increment = Eigen::Vector3d(0.0, 0.0, 0.1);
            break;
        case 'e': // translantion down
            linear_increment = Eigen::Vector3d(0.0, 0.0, -0.1);
            break;
        case '1': // rotation anti-clockwise (left turn)
            angular_increment =  0.05; 
            break;
        case '2': // rotation clockwise (right turn)
            angular_increment = -0.05;
            break;
        default:
            // unrecognized -> no movement
            break;
      }
    }

  void timerCallback(const ros::TimerEvent&)
    {
      // 1. update desired position & yaw
      xd += linear_increment;
      yawd += angular_increment;

      // LIMITATION
      double min_height = -10.0;
      double max_height = 10.0;
      // xd(2) = std::clamp(xd(2), min_height, max_height);
      xd(2) = std::max(min_height, std::min(xd(2), max_height));
      yawd = atan2(sin(yawd), cos(yawd));

      // 2. nav_msgs::Odometry
      // nav_msgs::Odometry des_odom;
      trajectory_msgs::MultiDOFJointTrajectory traj_msg;
      // des_odom.header.stamp = ros::Time::now();
      // des_odom.header.frame_id = "world";
      traj_msg.header.stamp = ros::Time::now();
      traj_msg.header.frame_id = "world"; // change? ######################################

      // position
      // des_odom.pose.pose.position.x = xd(0);
      // des_odom.pose.pose.position.y = xd(1);
      // des_odom.pose.pose.position.z = xd(2);

      // Create a trajectory point
      trajectory_msgs::MultiDOFJointTrajectoryPoint traj_point;
      // Set position and yaw
      geometry_msgs::Transform transform;
      transform.translation.x = xd(0);
      transform.translation.y = xd(1);
      transform.translation.z = xd(2);

      // yaw -> Quaternion
      tf2::Quaternion q_;
      q_.setRPY(0.0, 0.0, yawd);
      // des_odom.pose.pose.orientation = tf2::toMsg(q_);
      transform.rotation = tf2::toMsg(q_);

      // Add the transform to the trajectory point
      traj_point.transforms.push_back(transform);
      // Add the trajectory point to the message
      traj_msg.points.push_back(traj_point);
      
      // desired velocity、angular velocity
      // des_odom.twist.twist.linear.x = ...

      // 3. publish desired_state
      // desired_state.publish(des_odom);
      desired_state.publish(traj_msg);

      // linear_increment.setZero();
      // angular_increment = 0.0;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "manual_control_node");
  Manual_controlNode n;
  ros::spin();
  return 0;
}