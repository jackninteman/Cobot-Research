#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <memory>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

ros::Publisher joint1_torque_pub_;
ros::Publisher joint2_torque_pub_;
ros::Publisher joint3_torque_pub_;
ros::Publisher joint4_torque_pub_;
robot_state::RobotStatePtr kinematic_state;
robot_state::JointModelGroup* joint_model_group;
std::vector<std::string>* joint_names_ptr;

void ControlLawPublisher(const sensor_msgs::JointState::ConstPtr &jointStatesPtr_)
{ 
    
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group,joint_values);
    joint_values[0] = jointStatesPtr_->position[2];
    joint_values[1] = jointStatesPtr_->position[3];
    joint_values[2] = jointStatesPtr_->position[4];
    joint_values[3] = jointStatesPtr_->position[5];
    kinematic_state->setJointGroupPositions(joint_model_group,joint_values);
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    kinematic_state->getJacobian(joint_model_group,kinematic_state->getLinkModel("end_effector_link"),
      reference_point_position, jacobian);
    ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
    std_msgs::Float64 msg;
    msg.data = 10*(0.5 - jointStatesPtr_->position[2]) + 1*(0.0 - jointStatesPtr_->velocity[2]);
    
    joint1_torque_pub_.publish(msg);
    msg.data = 0.0;
    joint2_torque_pub_.publish(msg);
    joint3_torque_pub_.publish(msg);
    joint4_torque_pub_.publish(msg);
    
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "line_manipulation_control_law_publisher");
    std::unique_ptr<ros::NodeHandle> ros_node(new ros::NodeHandle);

    //Setup publisher to joint position action server
    joint1_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint1_position/command",1);
    joint2_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint2_position/command",1);
    joint3_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint3_position/command",1);
    joint4_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint4_position/command",1);
    
    //Setup subscriber to joint state controller
    ros::SubscribeOptions jointStateSubOption = ros::SubscribeOptions::create<sensor_msgs::JointState>(
        "/joint_states", 1, ControlLawPublisher, ros::VoidPtr(), ros_node->getCallbackQueue());
    ros::Subscriber jointStateSubscriber = ros_node->subscribe(jointStateSubOption);

    //Setup robot kinematics to calculate jacobian in the callback function
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state_temp(new robot_state::RobotState(kinematic_model));
    kinematic_state = std::move(kinematic_state_temp);
    kinematic_state->setToDefaultValues();
    joint_model_group = kinematic_model->getJointModelGroup("arm");
    std::vector<std::string> joint_names = joint_model_group->getVariableNames();
    joint_names_ptr = &joint_names;
    const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("end_effector_link");

    ros::spin();
    return 0;
}