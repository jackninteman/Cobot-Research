#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <memory>
//#include <moveit/robot_model_loader/robot_model_loader.h>
//#include <moveit/robot_model/robot_model.h>
//#include <moveit/robot_state/robot_state.h>
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/Dense>
#include "line3d.h"
#include "pseudo_inversion.h"

#include <ros/ros.h>

// Global variable declaration
ros::Publisher joint1_torque_pub_;
ros::Publisher joint2_torque_pub_;
ros::Publisher joint3_torque_pub_;
ros::Publisher joint4_torque_pub_;
ros::Publisher joint5_torque_pub_;
ros::Publisher joint6_torque_pub_;
ros::Publisher joint7_torque_pub_;
KDL::ChainDynParam* dyn_solver_raw_;
KDL::ChainJntToJacSolver* jac_solver_raw_;
KDL::ChainFkSolverPos_recursive* fk_solver_raw_;
KDL::Jacobian J_;
KDL::JntArray G_;
KDL::JntArray C_;
KDL::JntSpaceInertiaMatrix H_;
KDL::JntArray q_;
KDL::JntArray q_dot_;
KDL::Frame ee_tf_;

void ControlLawPublisher(const sensor_msgs::JointState::ConstPtr &jointStatesPtr_)
{ 
    // Explicit assignment for joint position and velocity
    q_.data[0] = jointStatesPtr_->position[2];
    q_.data[1] = jointStatesPtr_->position[3];
    q_.data[2] = jointStatesPtr_->position[4];
    q_.data[3] = jointStatesPtr_->position[5];
    q_.data[4] = jointStatesPtr_->position[6];
    q_.data[5] = jointStatesPtr_->position[7];
    q_.data[6] = jointStatesPtr_->position[8];
    q_dot_.data[0] = jointStatesPtr_->velocity[2];
    q_dot_.data[1] = jointStatesPtr_->velocity[3];
    q_dot_.data[2] = jointStatesPtr_->velocity[4];
    q_dot_.data[3] = jointStatesPtr_->velocity[5];
    q_dot_.data[4] = jointStatesPtr_->velocity[6];
    q_dot_.data[5] = jointStatesPtr_->velocity[7];
    q_dot_.data[6] = jointStatesPtr_->velocity[8];

    // Compute dynamics param, jacobian, and forward kinematics
    dyn_solver_raw_->JntToGravity(q_, G_);
    dyn_solver_raw_ ->JntToCoriolis(q_,q_dot_,C_);
    dyn_solver_raw_->JntToMass(q_,H_);
    jac_solver_raw_->JntToJac(q_, J_);
    fk_solver_raw_->JntToCart(q_,ee_tf_);

    // Compute mass matrix in cartesian space
    Eigen::MatrixXd mass_joint_inverse;
    Eigen::MatrixXd mass_cart;
    pseudoInverse(H_.data, mass_joint_inverse,false);
    //std::cout << "H_, H_inverse, H_*H_inverse, H_inverse*H_" << std::endl;
    //std::cout << H_.data << std::endl << std::endl << mass_joint_inverse << std::endl << std::endl;
    //std::cout << H_.data*mass_joint_inverse << std::endl << std::endl;
    //std::cout << mass_joint_inverse*H_.data << std::endl << std::endl;
    pseudoInverse(J_.data*mass_joint_inverse*J_.data.transpose(), mass_cart, false);
    
    // Compute Cholesky Decomposition
    Eigen::LLT<Eigen::MatrixXd> lltofmass_cart(mass_cart);
    Eigen::MatrixXd L = lltofmass_cart.matrixL();
    Eigen::MatrixXd L_inverse;
    pseudoInverse(L, L_inverse, false);

    // Put forward kinematics into proper forms
    Eigen::Vector3d ee_translation = Eigen::Map<Eigen::Vector3d>(ee_tf_.p.data);
    Eigen::Quaterniond ee_linear = Eigen::Map<Eigen::Quaterniond>(ee_tf_.M.data);

    // Setup line parameter
    std::vector<double> a = {0,1,0};
    std::vector<double> b = {0,1,0};
    std::vector<double> c = {0,0,0.3};
    //std::vector<double> a = {0,1,0};
    //std::vector<double> b = {0,1,0};
    //std::vector<double> c = {0,1,0};
    Line3d line(a,b,c);

    // Desired and current ee position and orientation
    Eigen::Vector3d desired_position(line.GetDesiredCrosstrackLocation(ee_translation));
    Eigen::Vector3d current_position(ee_translation);
    Eigen::Quaterniond desired_orientation(1,0,0,0);
    Eigen::Quaterniond current_orientation(ee_linear);
    //desired_position << 0.5, 0.5, 0.5;
    
    // Compute position error
    Eigen::Matrix<double,6,1> error;
    error.head(3) << desired_position - current_position;
    
    // Compute instantaneous frenet frame
    Eigen::Vector3d e_t(1/std::sqrt(2),1/std::sqrt(2),0);
    Eigen::Vector3d e_n(-error.head(3)/error.head(3).norm());
    Eigen::Vector3d e_b(e_n.cross(e_t));
    Eigen::Matrix<double,3,3> R3_3;
    R3_3 << e_t, e_b, e_n;
    Eigen::Matrix<double,6,6> R;
    R.setIdentity();
    R.topLeftCorner(3,3) << R3_3;
    R.bottomRightCorner(3,3) << R3_3;

    // Compute orientation error
    if(desired_orientation.coeffs().dot(current_orientation.coeffs()) < 0.0)
    {
        current_orientation.coeffs() << -current_orientation.coeffs();
    }
    Eigen::Quaterniond error_quaternion(current_orientation.inverse()*desired_orientation);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    error.tail(3) << ee_linear*error.tail(3);
    error.tail(3) << 0,0,0; //Don't care about orientation for now

    //Specify stiffness and damping
    Eigen::Matrix<double,6,6> cartesian_stiffness;
    Eigen::Matrix<double,6,6> cartesian_damping;
    cartesian_stiffness.setIdentity();
    cartesian_damping.setIdentity();
    cartesian_stiffness.topLeftCorner(3,3) << 500*Eigen::Matrix3d::Identity();
    cartesian_stiffness.bottomRightCorner(3,3) << 30*Eigen::Matrix3d::Identity();
    cartesian_damping.topLeftCorner(3,3) << 2*std::sqrt(500)*Eigen::Matrix3d::Identity();
    cartesian_damping.bottomRightCorner(3,3) << 2*std::sqrt(30)*Eigen::Matrix3d::Identity();

    // Specify adaptive damping
    Eigen::Matrix<double,6,6> K_des;
    K_des.setZero();
    K_des(0,0) = 0;
    K_des(1,1) = 0;
    K_des(2,2) = 800;

    // Compute K_bar
    Eigen::Matrix<double,6,6> K_bar(L_inverse*R*K_des*R.transpose()*L_inverse.transpose());
    
    // Compute eigenvalue and eigenvector
    Eigen::EigenSolver<Eigen::MatrixXd> es(K_bar);
    std::cout << "The eigenvalues:" << std::endl << es.eigenvalues().real() << std::endl;
    std::cout << "The eigenvector:" << std::endl << es.eigenvectors().real() << std::endl;
    Eigen::Matrix<double,6,6> S(es.eigenvectors().real());
    Eigen::Matrix<double,6,6> EV(es.eigenvalues().real().array().abs().sqrt().matrix().asDiagonal()*2.0*1.0);
    std::cout << "EV:" << std::endl << EV << std::endl;
    std::cout << "S:" << std::endl << S << std::endl;

    // Compute desired damping matrix
    Eigen::Matrix<double,6,6> C_des(R.transpose()*L*S*EV*S.transpose()*L.transpose()*R);
    C_des(0,0) = 0;
    C_des(1,1) = 0;
    std::cout << "C_des:" << std::endl << C_des << std::endl;
    
    //Compute Control Law
    //Eigen::Matrix<double,4,1> tau_d = J_.data.transpose()*(cartesian_stiffness*error + cartesian_damping*(-J_.data*q_dot_.data)) + G_.data + C_.data;
    Eigen::Matrix<double,4,1> tau_d = J_.data.transpose()*(R*K_des*R.transpose()*error + R*C_des*R.transpose()*(-J_.data*q_dot_.data)) + G_.data + C_.data;
    
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
    msg.data = tau_d[4];
    joint5_torque_pub_.publish(msg);
    msg.data = tau_d[5];
    joint6_torque_pub_.publish(msg);
    msg.data = tau_d[6];
    joint7_torque_pub_.publish(msg);
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
    joint1_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint1_effort/command",1);
    joint2_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint2_effort/command",1);
    joint3_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint3_effort/command",1);
    joint4_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint4_effort/command",1);
    joint5_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint5_effort/command",1);
    joint6_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint6_effort/command",1);
    joint7_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint7_effort/command",1);
    
    // Setup subscriber to joint state controller
    // Whenever we receive actual joint states, we will use callback function above to publish desired joint states
    ros::SubscribeOptions jointStateSubOption = ros::SubscribeOptions::create<sensor_msgs::JointState>(
        "/joint_states", 1, ControlLawPublisher, ros::VoidPtr(), ros_node->getCallbackQueue());
    ros::Subscriber jointStateSubscriber = ros_node->subscribe(jointStateSubOption);

    // Declare kdl stuff
    KDL::Tree 	kdl_tree_;
	KDL::Chain	kdl_chain_;
    std::unique_ptr<KDL::ChainDynParam> dyn_solver_;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    
    // Read robot description ros param
    std::string robot_desc_string;
    ros_node->param("robot_description", robot_desc_string, std::string());

    // kdl parser
    if (!kdl_parser::treeFromString(robot_desc_string, kdl_tree_))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }
    
    // Get kdl chain
    std::string root_name = "world";
	std::string tip_name = "panda_link7";
	  if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
      {
          ROS_ERROR("Failed to construct kdl chain");
          return false;
      }
    
    // This is a gravity 3D vector. Value is [0,0,-9.81]^T 
    KDL::Vector gravity_;
    gravity_ = KDL::Vector::Zero();
	gravity_(2) = -9.80;
    
    // Reset size of different matrices
    G_.resize(kdl_chain_.getNrOfJoints());
    H_.resize(kdl_chain_.getNrOfJoints());
    C_.resize(kdl_chain_.getNrOfJoints());
    J_.resize(kdl_chain_.getNrOfJoints());
    q_.resize(kdl_chain_.getNrOfJoints());
    q_dot_.resize(kdl_chain_.getNrOfJoints());
    
    // Inverse dynamics solver
	dyn_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
    jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    dyn_solver_raw_ = dyn_solver_.get();
    jac_solver_raw_ = jac_solver_.get();
    fk_solver_raw_ = fk_solver_.get();

    // Done with all the setup. Now wait in ros::spin until we get something from subscription
    ros::spin();
    return 0;
}