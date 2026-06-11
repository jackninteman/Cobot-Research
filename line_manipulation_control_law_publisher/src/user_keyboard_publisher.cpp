//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------

#include <cstdint>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
#include "cobot.h"
#include <ros/ros.h>

#include <Eigen/Dense>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include "std_msgs/UInt8.h"

#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

Eigen::Vector3d ext_force_keyboard(0, 0, 0);
ros::ServiceClient wrench_client;
gazebo_msgs::ApplyBodyWrench::Request apply_wrench_req;
gazebo_msgs::ApplyBodyWrench::Response apply_wrench_resp;

ros::Publisher f_ext_raw_pub_;

// Read one key with a timeout; returns '\0' if nothing pressed.
char getKey(double timeout_sec)
{
    struct termios old_t, raw_t;
    tcgetattr(STDIN_FILENO, &old_t);
    raw_t = old_t;
    raw_t.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &raw_t);

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    struct timeval tv;
    tv.tv_sec = (int)timeout_sec;
    tv.tv_usec = (int)((timeout_sec - (int)timeout_sec) * 1e6);

    char c = '\0';
    if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0)
    {
        if (read(STDIN_FILENO, &c, 1) < 0)
            c = '\0';
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &old_t);
    return c;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "user_controller");

    std::unique_ptr<ros::NodeHandle> nh(new ros::NodeHandle);

    f_ext_raw_pub_ = nh->advertise<geometry_msgs::Point>("/f_ext_raw", 10);

    // Setup apply body wrench service client
    wrench_client = nh->serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

    double max_force = 2;
    ros::Rate rate(30);

    while (ros::ok())
    {
        char k = getKey(1.0 / 30.0);
        if (k == '\x03')
            break; // Ctrl-C

        bool changed = true;
        switch (k)
        {
        case 'w':
            ext_force_keyboard[0] = 8;
            break; // +x
        case 's':
            ext_force_keyboard[0] = -8;
            break; // -x
        case 'a':
            ext_force_keyboard[1] = 8;
            break; // +y
        case 'd':
            ext_force_keyboard[1] = -8;
            break; // -y
        case 'q':
            ext_force_keyboard[2] = 8;
            break; // +z
        case 'e':
            ext_force_keyboard[2] = -8;
            break; // -z
        case ' ':
            ext_force_keyboard.setZero();
            break; // stop
        default:
            changed = false;
            break;
        }

        if (changed)
        {
            apply_wrench_req.body_name = "franka::panda_link7";
            apply_wrench_req.reference_frame = "world";
            apply_wrench_req.wrench.force.x = ext_force_keyboard[0] * max_force / 8;
            apply_wrench_req.wrench.force.y = ext_force_keyboard[1] * max_force / 8;
            apply_wrench_req.wrench.force.z = ext_force_keyboard[2] * max_force / 8;
            apply_wrench_req.duration = (ros::Duration)(-1);
            wrench_client.call(apply_wrench_req, apply_wrench_resp);
        }

        // Publish the current external force (same as before)
        Eigen::Vector3d Fext;
        Fext << apply_wrench_req.wrench.force.x, apply_wrench_req.wrench.force.y, apply_wrench_req.wrench.force.z;
        geometry_msgs::Point point_msg;
        point_msg.x = Fext(0);
        point_msg.y = Fext(1);
        point_msg.z = Fext(2);
        f_ext_raw_pub_.publish(point_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}