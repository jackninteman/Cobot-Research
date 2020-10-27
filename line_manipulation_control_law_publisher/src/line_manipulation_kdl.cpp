#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
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
ros::Publisher pose_pub_;
KDL::ChainDynParam* dyn_solver_raw_;
KDL::ChainJntToJacSolver* jac_solver_raw_;
KDL::ChainFkSolverPos_recursive* fk_solver_raw_;
KDL::ChainFkSolverPos_recursive* fk_solver_raw_2_;
KDL::Jacobian J_;
KDL::JntArray G_;
KDL::JntArray C_;
KDL::JntSpaceInertiaMatrix H_;
KDL::JntArray q_;
KDL::JntArray q_test_;
KDL::JntArray q_dot_;
KDL::Frame ee_tf_;
KDL::Frame some_link_;

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
    q_test_.data[0] = jointStatesPtr_->position[2];
    
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
    fk_solver_raw_2_->JntToCart(q_test_,some_link_);

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
    //Eigen::Quaterniond ee_linear = Eigen::Map<Eigen::Quaterniond>(ee_tf_.M.data);
    Eigen::Matrix3d rotation_matrix = Eigen::Map<Eigen::Matrix3d>(ee_tf_.M.data);
    Eigen::Quaterniond ee_linear(rotation_matrix.transpose());
    //Eigen::Quaterniond ee_linear(*(ee_tf_.M.data+3),*(ee_tf_.M.data),*(ee_tf_.M.data+1),*(ee_tf_.M.data+2));
    //Eigen::Matrix3d rotation_matrix = Eigen::Map<Eigen::Matrix3d>(some_link_.M.data);
    //Eigen::Quaterniond link_linear2(rotation_matrix.transpose());

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
    Eigen::Quaterniond desired_orientation(0.0, 1.0, 0.0, 0.0);
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
    //R.bottomRightCorner(3,3) << R3_3;

    // Compute orientation error
    if(desired_orientation.coeffs().dot(current_orientation.coeffs()) < 0.0)
    {
        current_orientation.coeffs() << -current_orientation.coeffs();
    }
    Eigen::Quaterniond error_quaternion(current_orientation.inverse()*desired_orientation);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    error.tail(3) << current_orientation*error.tail(3);
    //error.tail(3) << 0,0,0; //Don't care about orientation for now

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
    K_des(1,1) = 800;
    K_des(2,2) = 800;
    K_des(3,3) = 30;
    K_des(4,4) = 30;
    K_des(5,5) = 30;

    // Compute pseudoinverse for Jacobian
    Eigen::MatrixXd Jacobian_pseudoInverse_transpose;
    pseudoInverse(J_.data.transpose(),Jacobian_pseudoInverse_transpose,false);

    // Compute null space control law
    /*Eigen::Matrix<double,7,1> null_error;
    null_error.setZero();
    null_error(2) = 0.0 - q_.data[2];
    null_error(4) = 0.0 - q_.data[4];
    Eigen::Matrix<double,7,1> tau_null;
    Eigen::Matrix<double,7,7> identity_7by7;
    identity_7by7.setIdentity();
    tau_null = (identity_7by7 - J_.data.transpose()*Jacobian_pseudoInverse_transpose)*null_error*200;
    */

    // Compute K_bar
    Eigen::Matrix<double,6,6> K_bar(L_inverse*R*K_des*R.transpose()*L_inverse.transpose());
    std::cout << "K_bar: " << std::endl << K_bar << std::endl;

    // Compute eigenvalue and eigenvector
    Eigen::EigenSolver<Eigen::MatrixXd> es(K_bar);
    std::cout << "The eigenvalues:" << std::endl << es.eigenvalues() << std::endl;
    std::cout << "The eigenvector:" << std::endl << es.eigenvectors() << std::endl;
    Eigen::Matrix<double,6,6> S(es.eigenvectors().real());
    Eigen::Matrix<double,6,6> EV(es.eigenvalues().real().array().abs().sqrt().matrix().asDiagonal()*2.0*1.00);
    std::cout << "EV:" << std::endl << EV << std::endl;
    std::cout << "S:" << std::endl << S << std::endl;

    // Select only non zero eigenvales and eigenvectors
    Eigen::Matrix<double,6,6> EV_new;
    EV_new.setZero();
    Eigen::Matrix<double,6,6> S_new;
    S_new.setZero();
    int k=0;
    for (int i = 0; i < es.eigenvalues().real().size(); ++i)
    {
        if (es.eigenvalues().real()(i) > 0.001){
            EV_new(k,k) = std::sqrt(es.eigenvalues().real()(i))*2.0*1.00;
            S_new.col(k) = es.eigenvectors().real().col(i);
            ++k;
        } 
    }

    // Compute null space matrix
    Eigen::FullPivLU<Eigen::MatrixXd> lu(K_bar);
    Eigen::MatrixXd null_space = lu.kernel();
    std::cout << "null_space: " << std::endl << null_space << std::endl;

    // Compute orthogonal null space matrix
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(null_space);
    Eigen::MatrixXd null_space_orthogonal(qr.householderQ());
    std::cout << "null_space_orthogonal: " << std::endl << null_space_orthogonal << std::endl;
    
    for (int i = 0; i < null_space_orthogonal.cols(); ++i){
        S_new.col(k) = null_space_orthogonal.col(i);
        ++k;
        if (k > 6) break;
    }
    std::cout << "EV_new:" << std::endl << EV_new << std::endl;
    std::cout << "S_new:" << std::endl << S_new << std::endl;
    std::cout << "Testing S_new orthogonality:" << std::endl;
    std::cout << S_new.col(0).dot(S_new.col(1)) << std::endl;
    std::cout << S_new.col(0).dot(S_new.col(2)) << std::endl;
    std::cout << S_new.col(0).dot(S_new.col(3)) << std::endl;
    std::cout << S_new.col(0).dot(S_new.col(4)) << std::endl;
    std::cout << S_new.col(0).dot(S_new.col(5)) << std::endl;
    std::cout << S_new.col(1).dot(S_new.col(2)) << std::endl;
    std::cout << S_new.col(1).dot(S_new.col(3)) << std::endl;
    std::cout << S_new.col(1).dot(S_new.col(4)) << std::endl;
    std::cout << S_new.col(1).dot(S_new.col(5)) << std::endl;
    std::cout << S_new.col(2).dot(S_new.col(3)) << std::endl;
    std::cout << S_new.col(2).dot(S_new.col(4)) << std::endl;
    std::cout << S_new.col(2).dot(S_new.col(5)) << std::endl;
    std::cout << S_new.col(3).dot(S_new.col(4)) << std::endl;
    std::cout << S_new.col(3).dot(S_new.col(5)) << std::endl;
    std::cout << S_new.col(4).dot(S_new.col(5)) << std::endl;


    // Compute desired damping matrix
    Eigen::Matrix<double,6,6> C_des(R.transpose()*L*S*EV*S.transpose()*L.transpose()*R);
    //C_des(0,0) = 0;
    //C_des(1,1) = 0;
    std::cout << "C_des:" << std::endl << C_des << std::endl;
    Eigen::Matrix<double,6,6> C_des_new(R.transpose()*L*S_new*EV_new*S_new.transpose()*L.transpose()*R);
    //C_des(0,0) = 0;
    //C_des(1,1) = 0;
    std::cout << "C_des_new:" << std::endl << C_des_new << std::endl;
    std::cout << "mass_cart:" << std::endl << mass_cart << std::endl;
    //std::cout << "ST_Kbar_S:" << std::endl << S.transpose()*K_bar*S << std::endl;
    //std::cout << "ST_Kbar_S_new:" << std::endl << S_new.transpose()*K_bar*S_new << std::endl;
    //std::cout << "ST_Cbar_S:" << std::endl << S.transpose()*L_inverse*R*C_des*R.transpose()*L_inverse.transpose()*S << std::endl;
    //std::cout << "ST_Cbar_S_new:" << std::endl << S_new.transpose()*L_inverse*R*C_des_new*R.transpose()*L_inverse.transpose()*S_new << std::endl;
    
    //Compute Control Law
    //Eigen::Matrix<double,4,1> tau_d = J_.data.transpose()*(cartesian_stiffness*error + cartesian_damping*(-J_.data*q_dot_.data)) + G_.data + C_.data;
    Eigen::Matrix<double,7,1> tau_d = J_.data.transpose()*(R*K_des*R.transpose()*error + R*C_des_new*R.transpose()*(-J_.data*q_dot_.data)) + G_.data + C_.data;
    
    // Compute total control law
    //tau_d = tau_d + tau_null;
    //tau_d = tau_d;

    // Show R on screen
    std::cout << "R:" << std::endl << R << std::endl << std::endl;
    std::cout << current_orientation.w() << std::endl;
    std::cout << current_orientation.x() << std::endl;
    std::cout << current_orientation.y() << std::endl;
    std::cout << current_orientation.z() << std::endl;

    /*std::cout << *(some_link_.M.data) << std::endl;
    std::cout << *(some_link_.M.data+1) << std::endl;
    std::cout << *(some_link_.M.data+2) << std::endl;
    std::cout << *(some_link_.M.data+3) << std::endl;
    std::cout << *(some_link_.M.data+4) << std::endl;
    std::cout << *(some_link_.M.data+5) << std::endl;
    std::cout << *(some_link_.M.data+6) << std::endl;
    std::cout << *(some_link_.M.data+7) << std::endl;
    std::cout << *(some_link_.M.data+8) << std::endl << std::endl;
    std::cout << link_linear2.w() << std::endl;
    std::cout << link_linear2.x() << std::endl;
    std::cout << link_linear2.y() << std::endl;
    std::cout << link_linear2.z() << std::endl;*/

    //std::cout << current_orientation.norm() << std::endl;
    //std::cout << current_orientation.toRotationMatrix() << std::endl;
    /*std::cout << *ee_tf_.M.data << std::endl;
    std::cout << *(ee_tf_.M.data + 1)  << std::endl;
    std::cout << *(ee_tf_.M.data + 2) << std::endl;
    std::cout << *(ee_tf_.M.data + 3) << std::endl;*/

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

    //Publish Pose
    geometry_msgs::Pose pose;
    pose.position.x = current_position[0];
    pose.position.y = current_position[1];
    pose.position.z = current_position[2];
    pose.orientation.x = current_orientation.x();
    pose.orientation.y = current_orientation.y();
    pose.orientation.z = current_orientation.z();
    pose.orientation.w = current_orientation.w();
    pose_pub_.publish(pose);
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

    pose_pub_ = ros_node->advertise<geometry_msgs::Pose>("/pose",1);
    
    // Setup subscriber to joint state controller
    // Whenever we receive actual joint states, we will use callback function above to publish desired joint states
    ros::SubscribeOptions jointStateSubOption = ros::SubscribeOptions::create<sensor_msgs::JointState>(
        "/joint_states", 1, ControlLawPublisher, ros::VoidPtr(), ros_node->getCallbackQueue());
    ros::Subscriber jointStateSubscriber = ros_node->subscribe(jointStateSubOption);

    // Declare kdl stuff
    KDL::Tree 	kdl_tree_;
	KDL::Chain	kdl_chain_;
    KDL::Chain  kdl_chain_2_;
    std::unique_ptr<KDL::ChainDynParam> dyn_solver_;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_2_;
    
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
	std::string tip_name = "end_effector_link";
    std::string link_name = "panda_link1";
	  if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
      {
          ROS_ERROR("Failed to construct kdl chain");
          return false;
      }

      if(!kdl_tree_.getChain(root_name, link_name, kdl_chain_2_))
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
    q_test_.resize(kdl_chain_2_.getNrOfJoints());

    std::cout << "Number of joint: "<<kdl_chain_2_.getNrOfJoints() << std::endl;
    
    // Inverse dynamics solver
	dyn_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
    jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    fk_solver_2_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_2_));

    dyn_solver_raw_ = dyn_solver_.get();
    jac_solver_raw_ = jac_solver_.get();
    fk_solver_raw_ = fk_solver_.get();
    fk_solver_raw_2_ = fk_solver_2_.get();

    // Done with all the setup. Now wait in ros::spin until we get something from subscription
    ros::spin();
    return 0;
}