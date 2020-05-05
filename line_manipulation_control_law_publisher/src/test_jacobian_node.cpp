#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <memory>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_model_and_robot_state_tutorial");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    //Get Joint value
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group,joint_values);
    joint_values[0] = 0.1;
    joint_values[1] = -0.1;
    joint_values[2] = 0.1;
    joint_values[3] = 0.1;
    kinematic_state->setJointGroupPositions(joint_model_group,joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i){
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
    
    //Joint Limits
    
    //kinematic_state->setJointGroupPositions(joint_model_group,joint_values);
    ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
    //kinematic_state->enforceBounds();
    //ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

    //FW Kinematics
    //kinematic_state->setToRandomPositions(joint_model_group);
    const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("end_effector_link");
    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
    Eigen::Quaterniond quat(end_effector_state.linear());
    Eigen::Matrix<double,4,1> quaternion(quat.w(), quat.x(), quat.y(), quat.z());
    ROS_INFO_STREAM("Rotation: \n" << quaternion << "\n");

    std::cout << joint_model_group->getLinkModelNames().back() << std::endl;
    //Get Jacobian
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    /*kinematic_state->getJacobian(joint_model_group,kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
      reference_point_position, jacobian);
      */
    kinematic_state->getJacobian(joint_model_group,kinematic_state->getLinkModel("end_effector_link"),
      reference_point_position, jacobian); 
    ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
    ros::shutdown();
    return 0;
}
