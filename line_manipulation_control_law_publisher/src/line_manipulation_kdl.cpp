#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
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
#include "circle3d.h"
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
ros::Publisher traj_pub_;
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
Eigen::Vector3d p_initial(0.3,0.3,0.5);
Eigen::Vector3d p_final(-0.2,0.3,0.5);
Eigen::Vector3d p_center(0.0,0.0,0.5);
bool start_flag = true;
double time_begin_in_sec;
Eigen::Vector3d begin_cartesian_position;

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
    pseudoInverse(J_.data*mass_joint_inverse*J_.data.transpose(), mass_cart, false);
    
    // Compute Cholesky Decomposition
    Eigen::LLT<Eigen::MatrixXd> lltofmass_cart(mass_cart);
    Eigen::MatrixXd L = lltofmass_cart.matrixL();
    Eigen::MatrixXd L_inverse;
    pseudoInverse(L, L_inverse, false);

    // Put forward kinematics into proper forms
    Eigen::Vector3d ee_translation = Eigen::Map<Eigen::Vector3d>(ee_tf_.p.data);
    Eigen::Matrix3d rotation_matrix = Eigen::Map<Eigen::Matrix3d>(ee_tf_.M.data);
    // Need to transpose below for proper rotation matrix definition
    Eigen::Quaterniond ee_linear(rotation_matrix.transpose());

    // -----------Line/Circle Parameters----------------------------------------------------------
    // Setup line parameter
    /*std::vector<double> a = {0,1,0};
    std::vector<double> b = {0,1,0};
    std::vector<double> c = {0,0,0.3};
    Line3d line(a,b,c);*/
    Line3d line(p_initial, p_final);
    double distance_param;
    // Setup circle parameter
    //Circle3d circle(p_initial, p_final, p_center);
    //double theta_param;


    // ----------EE Pos: Set Desired and get current ee position and orientation-------------------
    Eigen::Vector3d desired_position(line.GetDesiredCrosstrackLocation(ee_translation,distance_param));
    //Eigen::Vector3d desired_position(circle.GetDesiredCrosstrackLocation(ee_translation,theta_param));
    Eigen::Vector3d current_position(ee_translation);
    Eigen::Quaterniond desired_orientation(0.0, 1.0, 0.0, 0.0); //(w,x,y,z) is the order shown here
    Eigen::Quaterniond current_orientation(ee_linear);
    

    // ----------EE Pos: Compute position and orientation error--------------------------------------
    Eigen::Matrix<double,6,1> error;
    error.head(3) << desired_position - current_position;
    if(desired_orientation.coeffs().dot(current_orientation.coeffs()) < 0.0)
    {
        current_orientation.coeffs() << -current_orientation.coeffs();
    }
    Eigen::Quaterniond error_quaternion(current_orientation.inverse()*desired_orientation);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    error.tail(3) << current_orientation*error.tail(3);
    //error.tail(3) << 0,0,0; //Don't care about orientation for now


    // -----------EE Twist: Set desired twist in frenet frame----------------------------------------
    Eigen::Matrix<double,6,1> desired_velocity_frenet;
    desired_velocity_frenet << 0.01, 0.0, 0.0, 0.0, 0.0, 0.0; //in order e_t,e_b,e_n,e_t,e_b,e_n
    

    // -----------Compute instantaneous frenet frame and rotation matrix------------------------------
    Eigen::Vector3d e_t(line.GetLineUnitDirection());
    //Eigen::Vector3d e_t(circle.GetUnitDirection(theta_param));
    Eigen::Vector3d e_n(-error.head(3)/error.head(3).norm());
    Eigen::Vector3d e_b(e_n.cross(e_t));
    Eigen::Matrix<double,3,3> R3_3;
    R3_3 << e_t, e_b, e_n;
    Eigen::Matrix<double,6,6> R;
    R.setIdentity();
    R.topLeftCorner(3,3) << R3_3;
    R.bottomRightCorner(3,3) << R3_3;

    
    /* ----------- Not currently using--------//Specify stiffness and damping
    Eigen::Matrix<double,6,6> cartesian_stiffness;
    Eigen::Matrix<double,6,6> cartesian_damping;
    cartesian_stiffness.setIdentity();
    cartesian_damping.setIdentity();
    cartesian_stiffness.topLeftCorner(3,3) << 500*Eigen::Matrix3d::Identity();
    cartesian_stiffness.bottomRightCorner(3,3) << 30*Eigen::Matrix3d::Identity();
    cartesian_damping.topLeftCorner(3,3) << 2*std::sqrt(500)*Eigen::Matrix3d::Identity();
    cartesian_damping.bottomRightCorner(3,3) << 2*std::sqrt(30)*Eigen::Matrix3d::Identity();
    */


    // -----------Specify cartesian stiffness------------------------------------------------------
    Eigen::Matrix<double,6,6> K_des;
    K_des.setZero();
    K_des(0,0) = 100;
    K_des(1,1) = 500;
    K_des(2,2) = 500;
    K_des(3,3) = 30;
    K_des(4,4) = 30;
    K_des(5,5) = 30;


    // -----------Compute pseudoinverse for Jacobian transpose-------------------------------------
    Eigen::MatrixXd Jacobian_pseudoInverse_transpose;
    pseudoInverse(J_.data.transpose(),Jacobian_pseudoInverse_transpose,false);

    
    // -----------Compute null space control law---------------------------------------------------
    // Below we are trying to make the elbow joint to stay at 0 and -pi/2 positions whenever possible
    // Need to revise this later but not important for now
    Eigen::Matrix<double,7,1> null_error;
    null_error.setZero();
    null_error(1) = 0.0 - q_.data[1];
    null_error(3) = -1.57 - q_.data[3];
    Eigen::Matrix<double,7,1> tau_null;
    Eigen::Matrix<double,7,7> identity_7by7;
    identity_7by7.setIdentity();
    tau_null = (identity_7by7 - J_.data.transpose()*Jacobian_pseudoInverse_transpose)*null_error*300.0; //if the gain is too high the whole thing can vibrate within the null space
    

    // -----------Compute K_bar (Positive definite K matrix normalized by mass matrix before diagonalized)
    Eigen::Matrix<double,6,6> K_bar(L_inverse*R*K_des*R.transpose()*L_inverse.transpose());
    //std::cout << "K_bar: " << std::endl << K_bar << std::endl;
    

    // -----------Compute eigenvalue and eigenvector of K_bar-------------------------------------------
    Eigen::EigenSolver<Eigen::MatrixXd> es(K_bar);
    //std::cout << "The eigenvalues:" << std::endl << es.eigenvalues() << std::endl;
    //std::cout << "The eigenvector:" << std::endl << es.eigenvectors() << std::endl;
    //////Eigen::Matrix<double,6,6> S(es.eigenvectors().real());
    // This eigenvalue is squared root before multiplied by 2 and zeta
    //////Eigen::Matrix<double,6,6> EV(es.eigenvalues().real().array().abs().sqrt().matrix().asDiagonal()*2.0*1.0);
    //std::cout << "EV:" << std::endl << EV << std::endl;
    //std::cout << "S:" << std::endl << S << std::endl;
    // Select only non zero eigenvales and eigenvectors
    Eigen::Matrix<double,6,6> EV;
    EV.setZero();
    Eigen::Matrix<double,6,6> S;
    S.setZero();
    int k=0; // set an index for non zero eigenvalues
    // Loop through to find non zero eigenvalue and asscosiated eigenvector
    for (int i = 0; i < es.eigenvalues().real().size(); ++i)
    {
        if (es.eigenvalues().real()(i) > 0.001){
            EV(k,k) = std::sqrt(es.eigenvalues().real()(i))*2.0*1.0;
            S.col(k) = es.eigenvectors().real().col(i);
            ++k;
        } 
    }
    //std::cout << "EV:" << std::endl << EV_new << std::endl;
    //std::cout << "S:" << std::endl << S_new << std::endl;

    // Compute null space matrix of K_bar (K_bar*v = 0, trying to find )
    Eigen::FullPivLU<Eigen::MatrixXd> lu(K_bar);
    Eigen::MatrixXd null_space = lu.kernel();
    //std::cout << "null_space: " << std::endl << null_space << std::endl;

    // Make the null_space matrix orthogonal
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(null_space);
    Eigen::MatrixXd null_space_orthogonal(qr.householderQ());
    //std::cout << "null_space_orthogonal: " << std::endl << null_space_orthogonal << std::endl;
    
    // Get a place holder for index for zero eigenvalue and associated eigenvectors
    int null_index = k;
    
    // Fill in to complete eigenvector matrix S
    for (int i = 0; i < null_space_orthogonal.cols(); ++i){
        if (k > 5) break;
        S.col(k) = null_space_orthogonal.col(i);
        ++k;
        if (k > 5) break;
    }
    
    // In case we want to check orthogonality of the S matrix
    /*std::cout << "EV_new:" << std::endl << EV_new << std::endl;
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
    */

    // ------------Compute desired damping matrix------------------------------------------------------------
    Eigen::Matrix<double,6,6> C_des(R.transpose()*L*S*EV*S.transpose()*L.transpose()*R);
    //std::cout << "C_des:" << std::endl << C_des << std::endl;

    
    //-------------Compute Control Law-------------------------------------------------------------------------
    //Eigen::Matrix<double,4,1> tau_d = J_.data.transpose()*(cartesian_stiffness*error + cartesian_damping*(-J_.data*q_dot_.data)) + G_.data + C_.data;
    Eigen::Matrix<double,7,1> tau_d;
    //tau_d = J_.data.transpose()*(R*K_des*R.transpose()*error + R*C_des*R.transpose()*(R*desired_velocity_frenet-J_.data*q_dot_.data)) + G_.data + C_.data;
    double time_in_sec = ros::Time::now().toSec();
    double time_begin_in_sec;
    std::cout << "Time now:" << time_in_sec << std::endl;
    if(time_in_sec > 120.0)
    {
        if(start_flag){
            time_begin_in_sec = time_in_sec;
            begin_cartesian_position = desired_position;
            start_flag = false;
        }
        time_in_sec = time_in_sec - time_begin_in_sec;
        Eigen::Matrix<double,3,1> desired_position_cartesian;
        desired_position_cartesian = line.GetDesiredPositionTrajectory(begin_cartesian_position,time_in_sec,5.0); 
        error.head(3) << desired_position_cartesian - current_position;
        
        Eigen::Matrix<double,6,1> desired_velocity_cartesian;
        desired_velocity_cartesian.setZero();
        desired_velocity_cartesian.head(3) << line.GetDesiredVelocityTrajectory(begin_cartesian_position,time_in_sec,5.0);

        Eigen::Matrix<double,6,1> desired_acceleration_cartesian;
        desired_acceleration_cartesian.setZero();
        desired_acceleration_cartesian.head(3) << line.GetDesiredAccelerationTrajectory(begin_cartesian_position,time_in_sec,5.0);
        
        // Publish Trajectory
        geometry_msgs::Point traj;
        traj.x = desired_position_cartesian[0];
        traj.y = desired_position_cartesian[1];
        traj.z = desired_position_cartesian[2];
        traj_pub_.publish(traj);

        // Control law for trajectory control
        tau_d = J_.data.transpose()*(mass_cart*desired_acceleration_cartesian+ R*K_des*R.transpose()*error + R*C_des*R.transpose()*(desired_velocity_cartesian-J_.data*q_dot_.data)) + G_.data + C_.data;
    }
    else
    {
        tau_d = J_.data.transpose()*(R*K_des*R.transpose()*error + R*C_des*R.transpose()*(-J_.data*q_dot_.data)) + G_.data + C_.data;
    }
    
    
    
    //-------------Compute total control law------------------------------------------------------------------
    tau_d = tau_d + tau_null;

    //-------------For debugging only-------------------------------------------------------------------------
    std::cout << "C_des:" << std::endl << C_des << std::endl;
    //std::cout << "mass_cart:" << std::endl << mass_cart << std::endl;
    //std::cout << "ST_Kbar_S:" << std::endl << S.transpose()*K_bar*S << std::endl;
    std::cout << "ST_Kbar_S:" << std::endl << S.transpose()*K_bar*S << std::endl;
    //std::cout << "ST_Cbar_S:" << std::endl << S.transpose()*L_inverse*R*C_des*R.transpose()*L_inverse.transpose()*S << std::endl;
    std::cout << "ST_Cbar_S:" << std::endl << S.transpose()*L_inverse*R*C_des*R.transpose()*L_inverse.transpose()*S << std::endl;
    
    // Show R on screen
    /*std::cout << "R:" << std::endl << R << std::endl << std::endl;
    std::cout << current_orientation.w() << std::endl;
    std::cout << current_orientation.x() << std::endl;
    std::cout << current_orientation.y() << std::endl;
    std::cout << current_orientation.z() << std::endl;
    std::cout << "Distance Parameter = " << distance_param << std::endl;
    std::cout << line.GetLineUnitDirection() << std::endl;
    */

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

    // Publish Pose
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

    traj_pub_ = ros_node->advertise<geometry_msgs::Point>("/traj",1);

    // Set rosparameter
    ros_node->setParam("/p_initial_x",p_initial[0]);
    ros_node->setParam("/p_initial_y",p_initial[1]);
    ros_node->setParam("/p_initial_z",p_initial[2]);
    ros_node->setParam("/p_final_x",p_final[0]);
    ros_node->setParam("/p_final_y",p_final[1]);
    ros_node->setParam("/p_final_z",p_final[2]);
    
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