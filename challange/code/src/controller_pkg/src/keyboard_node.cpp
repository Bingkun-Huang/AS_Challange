#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <stdio.h>
#include <termios.h>

class KeyboardTeleopNode {
public:
  KeyboardTeleopNode() : linear_scale_(0.1), angular_scale_(0.1) {
    pub_ = nh_.advertise<geometry_msgs::Twist>("keyboard_input", 1);
    initTermios();
  }

  ~KeyboardTeleopNode() { restoreTermios(); }

  void run() {
    ROS_INFO("Keyboard Teleop Node. Press Ctrl+C to exit.");
    ROS_INFO("Use the following keys to control the robot:");
    ROS_INFO("  i: Move forward");
    ROS_INFO("  k: Move backward");
    ROS_INFO("  l: Move right");
    ROS_INFO("  j: Move left");
    ROS_INFO("  u: Rotate counter-clockwise");
    ROS_INFO("  o: Rotate clockwise");
    ROS_INFO("  8: Move up");
    ROS_INFO("  9: Move down");

    while (ros::ok()) {
      char key = getKey();
      geometry_msgs::Twist twist;

      // Map keys to linear and angular velocities
      if (key == 'i') {
        twist.linear.y = 1.0;
        ROS_INFO("Moving forward");
      } else if (key == 'k') {
        twist.linear.y = -1.0;
        ROS_INFO("Moving backward");
      } else if (key == 'l') {
        twist.linear.x = 1.0;
        ROS_INFO("Moving right");
      } else if (key == 'j') {
        twist.linear.x = -1.0;
        ROS_INFO("Moving left");
      } else if (key == 'u') {
        twist.angular.z = 1.0;
        ROS_INFO("Rotating counter-clockwise");
      } else if (key == 'o') {
        twist.angular.z = -1.0;
        ROS_INFO("Rotating clockwise");
      } else if (key == '8') {
        twist.linear.z = 1.0;
        ROS_INFO("Moving up");
      } else if (key == '9') {
        twist.linear.z = -1.0;
        ROS_INFO("Moving down");
      } else {
        ROS_WARN("Invalid key pressed: %c", key);
        continue; // Skip publishing for invalid keys
      }

      // Publish the Twist message
      pub_.publish(twist);
      ros::spinOnce();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  double linear_scale_;
  double angular_scale_;
  struct termios initial_settings, new_settings;

  char getKey() {
    char key = 0;
    tcflush(0, TCIFLUSH);
    if (read(0, &key, 1) < 0) {
      perror("read():");
      ros::shutdown();
    }
    return key;
  }

  void initTermios() {
    tcgetattr(0, &initial_settings);
    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
  }

  void restoreTermios() { tcsetattr(0, TCSANOW, &initial_settings); }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "keyboard_teleop_node");
  KeyboardTeleopNode teleop_node;
  teleop_node.run();
  return 0;
}