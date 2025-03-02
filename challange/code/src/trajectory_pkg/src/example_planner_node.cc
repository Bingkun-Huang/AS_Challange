#include "ros/ros.h"
#include <example_planner.h>
#include <vector>

#include <iostream>

#include <chrono>
#include <queue>
#include <thread>

int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_planner");

  ExamplePlanner planner;
  ros::Duration(5.0).sleep();

  std::vector<double> x_list, y_list, z_list;
  planner.nh.getParam("/waypoints/x", x_list);
  planner.nh.getParam("/waypoints/y", y_list);
  planner.nh.getParam("/waypoints/z", z_list);

  std::string cur_mode;
  planner.nh.getParam("/params/mode", cur_mode);

  Eigen::Vector3d position, velocity;

  if (cur_mode == "auto") {

    for (int im = 0; im < x_list.size(); im++) {
      Eigen::Vector3d pre_defined_pos(x_list[im], y_list[im], z_list[im]);
      planner.pos_queue.push(pre_defined_pos);
    }

    while (true) {
      Eigen::Vector3d lastPoint(0.0, 0.0, 0.0);

      if (!planner.pos_queue.empty()) {
        lastPoint = planner.pos_queue.back();
        ROS_WARN("LAST POINT: %.3f, %.3f, %.3f", lastPoint.x(), lastPoint.y(),
                 lastPoint.z());
      }

      while (!planner.pos_queue.empty()) {

        ROS_WARN("NUM WAY POINTS: %lu",
                 (unsigned long)planner.pos_queue.size());

        position = planner.pos_queue.front();
        planner.his_queue.push(position);

        ROS_WARN("GET NEXT GOAL: %.3f, %.3f, %.3f", position.x(), position.y(),
                 position.z());

        std_msgs::Int64 his_length_msg;
        his_length_msg.data = planner.his_queue.size();

        planner.pub_length_his_queue_.publish(his_length_msg);

        planner.pos_queue.pop();
        ROS_WARN("REST WAY POINTS: %lu",
                 (unsigned long)planner.pos_queue.size());
        velocity << 0.0, 0.0, 0.0;

        for (int i = 0; i < 10; i++) {
          ros::spinOnce();
        }

        mav_trajectory_generation::Trajectory trajectory;

        if (planner.planTrajectory(position, velocity, &trajectory))
          ROS_WARN_STREAM("PLANNING DONE");
        if (planner.publishTrajectory(trajectory))
          ROS_WARN_STREAM("PUBLISHING TRAJ DONE");

        while (!(planner.isArrived(position))) {
          ros::spinOnce();
        }
      }

      if (planner.pos_queue.empty()) {

        ros::spinOnce();

        if (planner.isArrived(lastPoint)) {
          ROS_WARN_STREAM("ARRIVED");
          std_msgs::Bool msg;
          msg.data = true;
          planner.pub_state_waypoints_queue_.publish(msg);
        }
      }

      ros::Duration(1).sleep();
    }
  }
  return 0;
}