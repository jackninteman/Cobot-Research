#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <memory>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "line3d.h"

ros::Publisher joint1_torque_pub_;
ros::Publisher joint2_torque_pub_;
ros::Publisher joint3_torque_pub_;
ros::Publisher joint4_torque_pub_;
robot_state::RobotStatePtr kinematic_state;
robot_state::JointModelGroup* joint_model_group;
std::vector<std::string>* joint_names_ptr;
KDL::ChainDynParam* id_solver_raw_;
KDL::JntArray G_;
KDL::JntArray C_;
KDL::JntSpaceInertiaMatrix H_;
KDL::JntArray q_;
KDL::JntArray q_dot_;
KDL::Vector gravity_;
KDL::ChainJntToJacSolver* jac_solver_raw_;
KDL::Jacobian J_;

void ControlLawPublisher(const sensor_msgs::JointState::ConstPtr &jointStatesPtr_)
{ 
    // Setup for using FW kinematics and Jacobian
    // This function is run every time we receive current joint states
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group,joint_values);
    joint_values[0] = jointStatesPtr_->position[2];
    joint_values[1] = jointStatesPtr_->position[3];
    joint_values[2] = jointStatesPtr_->position[4];
    joint_values[3] = jointStatesPtr_->position[5];
    kinematic_state->setJointGroupPositions(joint_model_group,joint_values);
    Eigen::Matrix<double,4,1> joint_vel;
    joint_vel[0] = jointStatesPtr_->velocity[2];
    joint_vel[1] = jointStatesPtr_->velocity[3];
    joint_vel[2] = jointStatesPtr_->velocity[4];
    joint_vel[3] = jointStatesPtr_->velocity[5];

    q_.data = Eigen::Map<Eigen::Vector4d>(joint_values.data());
    q_dot_.data = joint_vel;
    
    
    

    id_solver_raw_->JntToGravity(q_, G_);
    id_solver_raw_ ->JntToCoriolis(q_,q_dot_,C_);
    id_solver_raw_->JntToMass(q_,H_);
    jac_solver_raw_->JntToJac(q_, J_);

    //Compute FW position kinematics
    const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("end_effector_link");
    //Eigen::Vector3d desired_position(0.288, 0, 0.193);
    Eigen::Vector3d current_position(end_effector_state.translation());
    //Setup line parameter
    std::vector<double> a = {0,1,0};
    std::vector<double> b = {0,0,0};
    std::vector<double> c = {0,0,0.12};
    //std::vector<double> a = {0,1,0};
    //std::vector<double> b = {0,1,0};
    //std::vector<double> c = {0,1,0};
    Line3d line(a,b,c);
    Eigen::Vector3d desired_position(line.GetDesiredCrosstrackLocation(end_effector_state.translation()));
    
    Eigen::Quaterniond desired_orientation(1,0,0,0);
    Eigen::Quaterniond current_orientation(end_effector_state.linear());
    
    //Compute position error
    Eigen::Matrix<double,6,1> error;
    error.head(3) << desired_position - current_position;
    
    //Compute orientation error
    if(desired_orientation.coeffs().dot(current_orientation.coeffs()) < 0.0)
    {
        current_orientation.coeffs() << -current_orientation.coeffs();
    }
    Eigen::Quaterniond error_quaternion(current_orientation.inverse()*desired_orientation);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    error.tail(3) << end_effector_state.linear()*error.tail(3);
    error.tail(3) << 0,0,0; //Don't care about orientation for now
    
    //Compute Jacobian
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    kinematic_state->getJacobian(joint_model_group,kinematic_state->getLinkModel("end_effector_link"), 
      reference_point_position, jacobian);

    //Specify stiffness and damping
    Eigen::Matrix<double,6,6> cartesian_stiffness;
    Eigen::Matrix<double,6,6> cartesian_damping;
    cartesian_stiffness.setIdentity();
    cartesian_damping.setIdentity();
    cartesian_stiffness.topLeftCorner(3,3) << 20*Eigen::Matrix3d::Identity();
    cartesian_stiffness.bottomRightCorner(3,3) << 2*Eigen::Matrix3d::Identity();
    cartesian_damping.topLeftCorner(3,3) << 8.94*Eigen::Matrix3d::Identity();
    cartesian_damping.bottomRightCorner(3,3) << 0.1*Eigen::Matrix3d::Identity();
    
    // KDL stuff



    //Compute Control Law
    //Eigen::Matrix<double,4,1> tau_d = jacobian.transpose()*(cartesian_stiffness*error + cartesian_damping*(-jacobian*joint_vel)) + G_.data + C_.data;
    Eigen::Matrix<double,4,1> tau_d = J_.data.transpose()*(cartesian_stiffness*error + cartesian_damping*(-jacobian*joint_vel)) + G_.data + C_.data;

    //Publish Control Law to each joint
    std_msgs::Float64 msg;
    msg.data = tau_d[0];
    joint1_torque_pub_.publish(msg);
     msg.data = tau_d[1];
    joint2_torque_pub_.publish(msg);
    msg.data = tau_d[2];
    joint3_torque_pub_.publish(msg);
    msg.data = tau_d[3];
    joint4_torque_pub_.publish(msg);

}

int main(int argc, char **argv)
{
    // Init this ros node
    ros::init(argc, argv, "line_manipulation_control_law_publisher");
    
    // Setup ros node handle as unique pointer pointed to heap section
    // This node handle will be there as long as the main function keeps spinning
    std::unique_ptr<ros::NodeHandle> ros_node(new ros::NodeHandle);

    // Setup publisher to joint position action server
    // The real publishing happens in the call back function above
    // Note that this publisher is declared globally above so that we can use these variables in the 
    // callback function
    joint1_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint1_position/command",1);
    joint2_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint2_position/command",1);
    joint3_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint3_position/command",1);
    joint4_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint4_position/command",1);
    
    // Setup subscriber to joint state controller
    // Whenever we receive actual joint states, we will use callback function above to publish desired joint states
    ros::SubscribeOptions jointStateSubOption = ros::SubscribeOptions::create<sensor_msgs::JointState>(
        "/joint_states", 1, ControlLawPublisher, ros::VoidPtr(), ros_node->getCallbackQueue());
    ros::Subscriber jointStateSubscriber = ros_node->subscribe(jointStateSubOption);

    // Setup robot kinematics to calculate jacobian in the callback function

    // Load robot model from rosparam robot_description
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

    // Get kinematic model pointer
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    // Get kinematic state pointer pointed to heap
    // We declare kinematic_state as a global variable so that we can use kinematic_state in the callback function above
    // Note that we cannot do this
    // robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    // because we declare kinematic_state as a global variable above
    // This is just a workaround to create a temp kinematic state and then move it to kinematic_state
    // [To Do] Maybe there is a better way to do this without using temp variable
    robot_state::RobotStatePtr kinematic_state_temp(new robot_state::RobotState(kinematic_model));
    kinematic_state = std::move(kinematic_state_temp);

    // Set kinematic_state to default value
    // Acutally this step is not required but a good practice so that kinematics is not at unknown states
    kinematic_state->setToDefaultValues();

    // Get joint model group named "arm"
    // This name is set up in SRDF by moveit!
    // Note that joint_model_group is a global variable
    joint_model_group = kinematic_model->getJointModelGroup("arm");
    
    // Get joint_names vector from joint_model_group
    std::vector<std::string> joint_names = joint_model_group->getVariableNames();
    joint_names_ptr = &joint_names;

    // Get 4x4 Homogeneous transformation called end_effector_state reference
    // That means the value in the reference can change over time
    const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("end_effector_link");

    // Set up tree
    KDL::Tree 	kdl_tree_;
	KDL::Chain	kdl_chain_;
    std::unique_ptr<KDL::ChainDynParam> id_solver_;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
    
    

    std::string robot_desc_string;
    ros_node->param("robot_description", robot_desc_string, std::string());

    // kdl parser
    if (!kdl_parser::treeFromString(robot_desc_string, kdl_tree_))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }
    
    // kdl chain
    std::string root_name = "world";
	std::string tip_name = "end_effector_link";
	  if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
      {
          ROS_ERROR("Failed to construct kdl chain");
          return false;
      }
    
    // This is a gravity 3D vector. Value is [0,0,-9.81]^T 
    gravity_ = KDL::Vector::Zero();
	gravity_(2) = -9.81;
    G_.resize(4);
    H_.resize(4);
    C_.resize(4);
    J_.resize(4);
    q_.resize(4);
    q_dot_.resize(4);
    //q_.data << 0,0,0,0;
    //q_dot_.data << 0.01,0.01,0.01,0.01;
    
    // inverse dynamics solver
	id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
    jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    id_solver_raw_ = id_solver_.get();
    jac_solver_raw_ = jac_solver_.get();

    // compute gravity torque
    /*
	std::cout << id_solver_raw_->JntToGravity(q_, G_) << std::endl;
    std::cout << "G_: " << G_.data << std::endl;

    std::cout << id_solver_raw_ ->JntToCoriolis(q_,q_dot_,C_);
    std::cout << "C_: " << C_.data << std::endl;
    
    std::cout << id_solver_raw_->JntToMass(q_,H_) << std::endl;
    std::cout << "H_:" << H_.data << std::endl;
    std::cout << "test" << std::endl;
    */
    // Done with all the setup. Now wait in ros::spin until we get something from subscription
    ros::spin();
    return 0;
}