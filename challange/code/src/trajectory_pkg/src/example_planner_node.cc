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

  std::vector<double> path_x, path_y, path_z;
  planner.nh.getParam("/waypoints/x", path_x);
  planner.nh.getParam("/waypoints/y", path_y);
  planner.nh.getParam("/waypoints/z", path_z);



  Eigen::Vector3d position, velocity;


    for (int i = 0; i < path_x.size(); i++) {
        Eigen::Vector3d path_to_cave(path_x[i], path_y[i], path_z[i]);
        planner.pos_queue.push(path_to_cave);
    }

    while (true) {
        Eigen::Vector3d lastPoint(0.0, 0.0, 0.0);

        if (!planner.pos_queue.empty()) {
        lastPoint = planner.pos_queue.back();
        }

        while (!planner.pos_queue.empty()) {

        position = planner.pos_queue.front();
        planner.his_queue.push(position);

        std_msgs::Int64 his_length_msg;
        his_length_msg.data = planner.his_queue.size();

        planner.pub_length_his_queue_.publish(his_length_msg);

        planner.pos_queue.pop();

        velocity << 0.0, 0.0, 0.0;

        for (int i = 0; i < 10; i++) {
            ros::spinOnce();
        }

        mav_trajectory_generation::Trajectory trajectory;

        planner.planTrajectory(position, velocity, &trajectory)
        planner.publishTrajectory(trajectory)

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
    
  return 0;
}