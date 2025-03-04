#include <example_planner.h>
#include <vector>

#include "ros/ros.h"
#include <iostream>

#include <chrono>
#include <queue>
#include <thread>

ExamplePlanner::ExamplePlanner()
    : max_v_(3), max_a_(1), isArrived_(false),
      current_velocity_(Eigen::Vector3d::Zero()),
      current_pose_(Eigen::Affine3d::Identity()) {

  if (!nh.getParam("/params/max_v", max_v_)) {
    ROS_WARN("[example_planner] param max_v not found");
  }

  if (!nh.getParam("/params/max_a", max_a_)) {
    ROS_WARN("[example_planner] param max_a not found");
  }

  pub_markers_ =
      nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);

  pub_trajectory_ =
      nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 0);

  pub_state_waypoints_queue_ =
      nh.advertise<std_msgs::Bool>("/state/waypoints_queue", 0);

  pub_length_his_queue_ = nh.advertise<std_msgs::Int64>("/length_his_queue", 1);

  sub_odom_ = nh.subscribe("current_state_est", 1,
                           &ExamplePlanner::uavOdomCallback, this);

  sub_waypoints_ = nh.subscribe<nav_msgs::Path>(
      "/visualization_marker", 1, &ExamplePlanner::onWaypoints, this);
}

void ExamplePlanner::onWaypoints(const nav_msgs::Path::ConstPtr &path) {
  for (const auto &it : path->poses) {
    Eigen::Vector3d new_pos(it.pose.position.x, it.pose.position.y,
                            it.pose.position.z);
    pos_queue.push(new_pos);
  }
}

void ExamplePlanner::uavOdomCallback(const nav_msgs::Odometry::ConstPtr &odom) {

  tf::poseMsgToEigen(odom->pose.pose, current_pose_);

  tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
}

void ExamplePlanner::setMaxSpeed(const double max_v) { max_v_ = max_v; }

bool ExamplePlanner::planTrajectory(
    const Eigen::VectorXd &goal_pos, const Eigen::VectorXd &goal_vel,
    mav_trajectory_generation::Trajectory *trajectory) {

  const int dimension = 3;

  mav_trajectory_generation::Vertex::Vector vertices;

  const int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::SNAP;

  mav_trajectory_generation::Vertex start(dimension), end(dimension);

  start.makeStartOrEnd(current_pose_.translation(), derivative_to_optimize);

  start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      current_velocity_);

  vertices.push_back(start);

  end.makeStartOrEnd(goal_pos, derivative_to_optimize);

  end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                    goal_vel);

  vertices.push_back(end);

  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

  mav_trajectory_generation::NonlinearOptimizationParameters parameters;

  const int N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension,
                                                                    parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  opt.addMaximumMagnitudeConstraint(
      mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
  opt.addMaximumMagnitudeConstraint(
      mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

  opt.optimize();

  opt.getTrajectory(&(*trajectory));

  return true;
}

bool ExamplePlanner::publishTrajectory(
    const mav_trajectory_generation::Trajectory &trajectory) {
  visualization_msgs::MarkerArray markers;
  double distance = 5.0; // Distance by which to seperate additional markers.
                         // Set 0.0 to disable.
  std::string frame_id = "world";

  mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id,
                                               &markers);
  pub_markers_.publish(markers);

  mav_planning_msgs::PolynomialTrajectory msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,
                                                                 &msg);
  msg.header.frame_id = "world";
  pub_trajectory_.publish(msg);

  return true;
}


bool ExamplePlanner::isArrived(const Eigen::VectorXd &goal_pos) {

  double distance = 0;

  double current_x = current_pose_.translation().x();
  double current_y = current_pose_.translation().y();
  double current_z = current_pose_.translation().z();

  double goal_x = goal_pos.x();
  double goal_y = goal_pos.y();
  double goal_z = goal_pos.z();

  distance =
      std::sqrt(std::pow(goal_x - current_x, 2) + std::pow(goal_y - current_y, 2) +
                std::pow(goal_z - current_z, 2));

  if (distance > 3.0) {
    return false;
  }

  return true;
}

bool ExamplePlanner::isIdle() {

  double current_velocity_x = current_velocity_.x();
  double current_velocity_y = current_velocity_.y();
  double current_velocity_z = current_velocity_.z();

  double current_velocity =
      sqrt(std::pow(current_velocity_x, 2) + std::pow(current_velocity_y, 2) +
      std::pow(current_velocity_z, 2));
      
  if (current_velocity > 0.001) {
    return false;
  }

  return true;
}