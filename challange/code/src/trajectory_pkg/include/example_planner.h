#ifndef MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H
#define MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H

#include <iostream>
#include <queue>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Int64.h"
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

class ExamplePlanner {
 public:

  ros::NodeHandle nh;
  
  std::queue<Eigen::Vector3d> pos_queue; // queue for poses to excute

  std::queue<Eigen::Vector3d> his_queue; // queue for excuted poses



  ros::Publisher pub_state_waypoints_queue_;
  ros::Publisher pub_length_his_queue_;
  

  ExamplePlanner();

  void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose);

  void onWaypoints(const nav_msgs::Path::ConstPtr& path);

  void setMaxSpeed(double max_v);

  // Plans a trajectory to take off from the current position and
  // fly to the given altitude (while maintaining x,y, and yaw).
  bool planTrajectory(const Eigen::VectorXd& goal_pos,
                      const Eigen::VectorXd& goal_vel,
                      mav_trajectory_generation::Trajectory* trajectory);
                      
  bool planTrajectory(const Eigen::VectorXd& goal_pos,
                      const Eigen::VectorXd& goal_vel,
                      const Eigen::VectorXd& start_pos,
                      const Eigen::VectorXd& start_vel,
                      double v_max, double a_max,
                      mav_trajectory_generation::Trajectory* trajectory);
                      
  bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);

  bool isArrived(const Eigen::VectorXd& goal_pos);

  bool isIdle();

 private:
  ros::Publisher pub_markers_;
  ros::Publisher pub_trajectory_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_waypoints_;

  
  Eigen::Affine3d current_pose_;
  Eigen::Vector3d current_velocity_;
  Eigen::Vector3d current_angular_velocity_;
  double max_v_; // m/s
  double max_a_; // m/s^2
  double max_ang_v_;
  double max_ang_a_;
  bool isArrived_;

};

#endif // MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H
