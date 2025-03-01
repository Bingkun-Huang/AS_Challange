#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

char getKey()
{
    char key;
    struct termios oldt, newt;

    // Save the terminal settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Disable canonical mode and echo
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // Read a single character
    key = getchar();

    // Restore the terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return key;
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "keyboard_input_publisher");
    ros::NodeHandle nh;

    // Publisher to the /keyboard_input topic
    ros::Publisher keyboard_pub = nh.advertise<std_msgs::UInt8>("keyboard_input", 10);

    ros::Rate rate(10); // 10 Hz

    std::cout << "Press keys to control. Press 'o' to quit." << std::endl;

    while (ros::ok())
    {
        try
        {
            // Get key input
            char key = getKey();

            // Quit on 'o'
            if (key == 'o')
            {
                std::cout << "Exiting..." << std::endl;
                break;
            }

            // Publish the key as a ROS message
            // std_msgs::String msg;
            std_msgs::UInt8 msg;
            // msg.data = std::string(1, key);
            msg.data = static_cast<uint8_t>(key);
            keyboard_pub.publish(msg);

            ROS_INFO("Key '%c' published to keyboard_input", key);
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("Error: %s", e.what());
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}