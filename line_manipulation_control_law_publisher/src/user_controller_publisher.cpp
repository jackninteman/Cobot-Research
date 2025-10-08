//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------

#include <cstdint>
#include <cmath>
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <vector>
#include "cobot.h"
#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <Eigen/Dense>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float64.h"
#include <ros/callback_queue.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

Eigen::Vector3d ext_force_joystick;
ros::ServiceClient wrench_client;
gazebo_msgs::ApplyBodyWrench::Request apply_wrench_req;
gazebo_msgs::ApplyBodyWrench::Response apply_wrench_resp;

std::vector<uint8_t> hybrid_mode_list = DEFAULT_MODE;
int desired_mode_idx = LINE_MODE_IDX;

// Declare callback function to handle user controller input
void JoystickFeedback(const sensor_msgs::Joy::ConstPtr &joystickhandlePtr_);

ros::Publisher hybrid_mode_pub_;
ros::Publisher f_ext_raw;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "user_controller");

    // Setup nh as unique pointer pointed to heap section
    // This node handle will be there as long as the main function keeps spinning
    std::unique_ptr<ros::NodeHandle> nh(new ros::NodeHandle);

    // Setup subscriber to joy ps4 controller
    ros::SubscribeOptions joystickSubOption = ros::SubscribeOptions::create<sensor_msgs::Joy>(
        "/joy", 1, JoystickFeedback, ros::VoidPtr(), nh->getCallbackQueue());
    ros::Subscriber joystickSubscriber = nh->subscribe(joystickSubOption);

    hybrid_mode_pub_ = nh->advertise<std_msgs::UInt8MultiArray>("/hybrid_mode", 1);

    f_ext_raw = nh->advertise<geometry_msgs::Point>("/f_ext_raw", 10);

    // Setup apply body wrench service client
    wrench_client = nh->serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

    ros::Rate rate(30);

    while (ros::ok())
    {
        Eigen::Vector4d Fext_homogeneous;
        Fext_homogeneous << apply_wrench_req.wrench.force.x, apply_wrench_req.wrench.force.y, apply_wrench_req.wrench.force.z, 1.0;
        geometry_msgs::Point point_msg;
        point_msg.x = Fext_homogeneous(0);
        point_msg.y = Fext_homogeneous(1);
        point_msg.z = Fext_homogeneous(2);

        f_ext_raw.publish(point_msg);

        // Always publish the current hybrid mode
        std_msgs::UInt8MultiArray hybrid_mode;
        hybrid_mode.data = hybrid_mode_list;
        hybrid_mode_pub_.publish(hybrid_mode);

        ros::spinOnce();
        rate.sleep();
    }
}

void JoystickFeedback(const sensor_msgs::Joy::ConstPtr &joystickhandlePtr_)
{
    ext_force_joystick[0] = (joystickhandlePtr_->axes[0]) * (-1.0); // force along x-axis. Left analog controller going left/right
    ext_force_joystick[1] = (joystickhandlePtr_->axes[2]) * (-1.0); // force along y-axis. Right analog controller going left/right
    ext_force_joystick[2] = joystickhandlePtr_->axes[1];            // force along z-axis. Left analog controller going up/down
    bool buttonX = joystickhandlePtr_->buttons[0];                  // X button
    bool buttonA = joystickhandlePtr_->buttons[1];                  // A button
    bool buttonB = joystickhandlePtr_->buttons[2];                  // B button
    bool buttonY = joystickhandlePtr_->buttons[3];                  // Y button
    bool buttonR2 = joystickhandlePtr_->buttons[7];                 // R2 button

    double max_force = 20;
    apply_wrench_req.body_name = "franka::panda_link7";
    apply_wrench_req.reference_frame = "world";
    apply_wrench_req.wrench.force.x = ext_force_joystick[0] * max_force / 8;
    apply_wrench_req.wrench.force.y = ext_force_joystick[1] * max_force / 8;
    apply_wrench_req.wrench.force.z = ext_force_joystick[2] * max_force / 8;
    // apply_wrench_req.start_time = 0;
    apply_wrench_req.duration = (ros::Duration)(-1);
    wrench_client.call(apply_wrench_req, apply_wrench_resp);

    // Handle hybrid mode switching
    if (buttonA)
    {
        // LINE
        desired_mode_idx = LINE_MODE_IDX;
    }
    else if (buttonB)
    {
        // PLANE
        desired_mode_idx = PLANE_MODE_IDX;
    }
    else if (buttonY)
    {
        // CIRCLE
        desired_mode_idx = CIRCLE_MODE_IDX;
    }
    else if (buttonX)
    {
        // SPLINE
        desired_mode_idx = SPLINE_MODE_IDX;
    }
    // Update hybrid mode list based on button inputs
    for (int i = 0; i < NUM_MODES; i++)
    {
        if (i == desired_mode_idx)
        {
            hybrid_mode_list[i] = 1;
        }
        else
        {
            hybrid_mode_list[i] = 0;
        }
    }

    // Always publish the current hybrid mode
    std_msgs::UInt8MultiArray hybrid_mode;
    hybrid_mode.data = hybrid_mode_list;
    hybrid_mode_pub_.publish(hybrid_mode);
}