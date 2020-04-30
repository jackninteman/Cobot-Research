#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <memory>

ros::Publisher joint1_torque_pub_;
ros::Publisher joint2_torque_pub_;
ros::Publisher joint3_torque_pub_;
ros::Publisher joint4_torque_pub_;

void ControlLawPublisher(const sensor_msgs::JointState::ConstPtr &jointStatesPtr_)
{
    std_msgs::Float64 msg;
    //msg.data = jointStatesPtr_->position[2];
    msg.data = 0.0;
    joint1_torque_pub_.publish(msg);
    //msg.data = jointStatesPtr_->position[3];
    joint2_torque_pub_.publish(msg);
    //msg.data = jointStatesPtr_->position[4];
    joint3_torque_pub_.publish(msg);
    //msg.data = jointStatesPtr_->position[5];
    joint4_torque_pub_.publish(msg);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "line_manipulation_control_law_publisher");
    std::unique_ptr<ros::NodeHandle> ros_node(new ros::NodeHandle);
    joint1_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint1_position/command",1);
    joint2_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint2_position/command",1);
    joint3_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint3_position/command",1);
    joint4_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint4_position/command",1);
    ros::SubscribeOptions jointStateSubOption = ros::SubscribeOptions::create<sensor_msgs::JointState>(
        "/joint_states", 1, ControlLawPublisher, ros::VoidPtr(), ros_node->getCallbackQueue());
    
    ros::Subscriber jointStateSubscriber = ros_node->subscribe(jointStateSubOption);
    ros::spin();
    return 0;
}