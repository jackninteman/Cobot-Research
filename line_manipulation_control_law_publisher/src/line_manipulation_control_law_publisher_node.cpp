#include "ros/ros.h"
#include "std_msgs/Float64.h"
///joint2_position/command

int main(int argc, char **argv)
{
    ros::init(argc, argv, "line_manipulation_control_law_publisher");
    ros::NodeHandle n;
    ros::Publisher joint1_torque_pub = n.advertise<std_msgs::Float64>("/joint1_position/command",1000);
    ros::Publisher joint2_torque_pub = n.advertise<std_msgs::Float64>("/joint2_position/command",1000);
    ros::Publisher joint3_torque_pub = n.advertise<std_msgs::Float64>("/joint3_position/command",1000);
    ros::Publisher joint4_torque_pub = n.advertise<std_msgs::Float64>("/joint4_position/command",1000);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        std_msgs::Float64 msg;
        msg.data = 0.0;
        joint1_torque_pub.publish(msg);
        joint2_torque_pub.publish(msg);
        joint3_torque_pub.publish(msg);
        joint4_torque_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();

    }
}