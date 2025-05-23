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
#include <std_msgs/Float64.h>
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "space_manipulation/line3d.h"
#include "space_manipulation/circle3d.h"
#include "space_manipulation/plane3d.h"
#include "space_manipulation/spline3d.h"
#include "pseudo_inversion.h"
#include <ros/ros.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <visualization_msgs/Marker.h>

//------------------------------------------------------------------------------
// DEFINES
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// VARIABLES
//------------------------------------------------------------------------------

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
ros::Publisher twist_pub_;
ros::Publisher hybrid_mode_pub_;
ros::Publisher matrix_pub;
ros::Publisher spline_pub;
ros::ServiceClient wrench_client;
gazebo_msgs::ApplyBodyWrench::Request apply_wrench_req;
gazebo_msgs::ApplyBodyWrench::Response apply_wrench_resp;
KDL::ChainDynParam *dyn_solver_raw_;
KDL::ChainJntToJacSolver *jac_solver_raw_;
KDL::ChainFkSolverPos_recursive *fk_solver_raw_;
KDL::ChainFkSolverPos_recursive *fk_solver_force_frame_raw_;
KDL::Jacobian J_;
KDL::JntArray G_;
KDL::JntArray C_;
KDL::JntSpaceInertiaMatrix H_;
KDL::JntArray q_;
KDL::JntArray q_dot_;
KDL::JntArray effort_;
KDL::Frame ee_tf_;
KDL::Frame force_frame_tf_;
Eigen::Vector3d ext_force_body;
Eigen::Vector3d ext_force_joystick;

// Points for line and plane
Eigen::Vector3d p_1(0.5, -0.25, 0.15);
Eigen::Vector3d p_2(0.3, 0.25, 0.3);
Eigen::Vector3d p_3(0.5, 0.0, 0.305);
// Points for circle
Eigen::Vector3d p_c1(0.38, 0.25, 0.5);
Eigen::Vector3d p_c2(0.35, 0.0, 0.75);
Eigen::Vector3d p_cc(0.4, 0.0, 0.5);
// Points for spline
Eigen::Vector3d p_s1(0.5, -0.5, 0.1);
Eigen::Vector3d p_s2(0.5, -0.25, 0.25);
Eigen::Vector3d p_s3(0.5, 0.2, 0.3);
Eigen::Vector3d p_s4(0.5, 0.5, 0.5);

bool start_flag = true;
double time_begin_in_sec;
Eigen::Vector3d begin_cartesian_position;

std::vector<uint8_t> hybrid_mode_list = DEFAULT_MODE;
int desired_mode_idx = 0;

// Parameters----------------------------------------------------------
#ifdef HYBRID
// Setup line, plane, and circle parameters
Line3d line(p_1, p_2);
double distance_param;
Plane3d plane(p_1, p_2, p_3);
double distance_param_x, distance_param_y;
Circle3d circle(p_c1, p_c2, p_cc);
double theta_param;
Spline3d spline(p_s1, p_s2, p_s3, p_s4);
#endif

//------------------------------------------------------------------------------
// FUNCTIONS
//------------------------------------------------------------------------------

void publishMatrices(const Eigen::MatrixXd &S, const Eigen::MatrixXd &L);

void JoystickFeedback(const sensor_msgs::Joy::ConstPtr &joystickhandlePtr_);

void ForceSensorPublisher(const geometry_msgs::WrenchStamped::ConstPtr &forceSensorPtr_);

void ControlLawPublisher(const sensor_msgs::JointState::ConstPtr &jointStatesPtr_);

uint8_t CheckOctant(Eigen::Vector3d e_t);

int main(int argc, char **argv)
{
    // Init this ros node
    ros::init(argc, argv, "line_manipulation_control_law_publisher");

    // Setup ros node handle as unique pointer pointed to heap section
    // This node handle will be there as long as the main function keeps spinning
    std::unique_ptr<ros::NodeHandle> ros_node(new ros::NodeHandle);

    // Initialize the publisher to publish on the "matrix_topic" topic
    matrix_pub = ros_node->advertise<std_msgs::Float64MultiArray>("matrix_topic", 10);

    // Setup publisher to joint position action server
    // The real publishing happens in the call back function above
    // Note that this publisher is declared globally above so that we can use these variables in the
    // callback function
    joint1_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint1_effort/command", 1);
    joint2_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint2_effort/command", 1);
    joint3_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint3_effort/command", 1);
    joint4_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint4_effort/command", 1);
    joint5_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint5_effort/command", 1);
    joint6_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint6_effort/command", 1);
    joint7_torque_pub_ = ros_node->advertise<std_msgs::Float64>("/joint7_effort/command", 1);

    pose_pub_ = ros_node->advertise<geometry_msgs::Pose>("/pose", 1);

    traj_pub_ = ros_node->advertise<geometry_msgs::Point>("/traj", 1);
    twist_pub_ = ros_node->advertise<geometry_msgs::Twist>("/twist", 1);

    hybrid_mode_pub_ = ros_node->advertise<std_msgs::UInt8MultiArray>("/hybrid_mode", 1);

    spline_pub = ros_node->advertise<visualization_msgs::Marker>("/spline", 1);

    // Set rosparameters for line
    ros_node->setParam("/p_initial_x", p_1[0]);
    ros_node->setParam("/p_initial_y", p_1[1]);
    ros_node->setParam("/p_initial_z", p_1[2]);
    ros_node->setParam("/p_final_x", p_2[0]);
    ros_node->setParam("/p_final_y", p_2[1]);
    ros_node->setParam("/p_final_z", p_2[2]);
    // Set rosparameters for plane
    Plane3d plane(p_1, p_2, p_3);
    Eigen::Quaterniond plane_orientation(plane.GetPlaneUnitDirection());
    ros_node->setParam("/plane_orientation_w", plane_orientation.w());
    ros_node->setParam("/plane_orientation_x", plane_orientation.x());
    ros_node->setParam("/plane_orientation_y", plane_orientation.y());
    ros_node->setParam("/plane_orientation_z", plane_orientation.z());
    // Set rosparameters for circle
    ros_node->setParam("/p_center_x", p_cc[0]);
    ros_node->setParam("/p_center_y", p_cc[1]);
    ros_node->setParam("/p_center_z", p_cc[2]);
    Circle3d circle(p_c1, p_c2, p_cc);
    Eigen::MatrixXd R_circle = circle.GetRot();
    // Must flatten the R matrix so it can be set as a rosparameter
    std::vector<double> R_flat = {R_circle(0, 0), R_circle(0, 1), R_circle(0, 2),
                                  R_circle(1, 0), R_circle(1, 1), R_circle(1, 2),
                                  R_circle(2, 0), R_circle(2, 1), R_circle(2, 2)};
    ros_node->setParam("/circle_rad", circle.GetRadius());
    ros_node->setParam("/circle_rot", R_flat);

    // Setup subscriber to joint state controller
    // Whenever we receive actual joint states, we will use callback function above to publish desired joint states
    ros::SubscribeOptions jointStateSubOption = ros::SubscribeOptions::create<sensor_msgs::JointState>(
        "/joint_states", 1, ControlLawPublisher, ros::VoidPtr(), ros_node->getCallbackQueue());
    ros::Subscriber jointStateSubscriber = ros_node->subscribe(jointStateSubOption);
    /*message_filters::Subscriber<sensor_msgs::JointState> jointStateSubscriber(*ros_node, "/joint_states", 1);
    message_filters::Subscriber<geometry_msgs::WrenchStamped> forceSensorSubscriber(*ros_node, "/sensor_panda_joint7", 1);
    message_filters::TimeSynchronizer<sensor_msgs::JointState, geometry_msgs::WrenchStamped> sync(jointStateSubscriber, forceSensorSubscriber, 10);
    sync.registerCallback(boost::bind(&ControlLawPublisher, _1, _2));*/

    // Setup subscriber to force sensor
    ros::SubscribeOptions forceSensorSubOption = ros::SubscribeOptions::create<geometry_msgs::WrenchStamped>(
        "/sensor_panda_joint7", 1, ForceSensorPublisher, ros::VoidPtr(), ros_node->getCallbackQueue());
    ros::Subscriber forceSensorSubscriber = ros_node->subscribe(forceSensorSubOption);

    // Setup subscriber to joy ps4 controller
    ros::SubscribeOptions joystickSubOption = ros::SubscribeOptions::create<sensor_msgs::Joy>(
        "/joy", 1, JoystickFeedback, ros::VoidPtr(), ros_node->getCallbackQueue());
    ros::Subscriber joystickSubscriber = ros_node->subscribe(joystickSubOption);

    // Setup apply body wrench service client
    wrench_client = ros_node->serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

    // Declare kdl stuff
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;
    KDL::Chain kdl_chain_force_frame_;
    std::unique_ptr<KDL::ChainDynParam> dyn_solver_;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_force_frame_;

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
    std::string force_frame_name = "panda_link7";
    if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
    {
        ROS_ERROR("Failed to construct kdl chain");
        return false;
    }
    if (!kdl_tree_.getChain(root_name, force_frame_name, kdl_chain_force_frame_))
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
    effort_.resize(kdl_chain_.getNrOfJoints());

    // Inverse dynamics solver
    dyn_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
    jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    fk_solver_force_frame_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_force_frame_));

    dyn_solver_raw_ = dyn_solver_.get();
    jac_solver_raw_ = jac_solver_.get();
    fk_solver_raw_ = fk_solver_.get();
    fk_solver_force_frame_raw_ = fk_solver_force_frame_.get();

    // Done with all the setup. Now wait in ros::spin until we get something from subscription
    ros::spin();
    return 0;
}

void ControlLawPublisher(const sensor_msgs::JointState::ConstPtr &jointStatesPtr_)
{
    // Explicit assignment for joint position, velocity, and effort
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

    effort_.data[0] = jointStatesPtr_->effort[2];
    effort_.data[1] = jointStatesPtr_->effort[3];
    effort_.data[2] = jointStatesPtr_->effort[4];
    effort_.data[3] = jointStatesPtr_->effort[5];
    effort_.data[4] = jointStatesPtr_->effort[6];
    effort_.data[5] = jointStatesPtr_->effort[7];
    effort_.data[6] = jointStatesPtr_->effort[8];

    /*Eigen::Vector3d ext_force_body_frame;
    ext_force_body_frame[0] = forceSensorPtr_-> wrench.force.x;
    ext_force_body_frame[1] = forceSensorPtr_-> wrench.force.y;
    ext_force_body_frame[2] = forceSensorPtr_-> wrench.force.z;
    */

    // Get time now
    double time_in_sec = ros::Time::now().toSec();

    // Compute dynamics param, jacobian, and forward kinematics
    dyn_solver_raw_->JntToGravity(q_, G_);
    dyn_solver_raw_->JntToCoriolis(q_, q_dot_, C_);
    dyn_solver_raw_->JntToMass(q_, H_);
    jac_solver_raw_->JntToJac(q_, J_);
    fk_solver_raw_->JntToCart(q_, ee_tf_);
    fk_solver_force_frame_raw_->JntToCart(q_, force_frame_tf_);

    // Compute mass matrix in cartesian space
    Eigen::MatrixXd mass_joint_inverse;
    Eigen::MatrixXd mass_cart;
    pseudoInverse(H_.data, mass_joint_inverse, false);
    pseudoInverse(J_.data * mass_joint_inverse * J_.data.transpose(), mass_cart, false);

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
    Eigen::Matrix3d force_rotation_matrix = Eigen::Map<Eigen::Matrix3d>(force_frame_tf_.M.data);
    force_rotation_matrix = force_rotation_matrix.transpose();

    // ----------EE Pos: Set Desired and get current ee position and orientation-------------------
    // 3) Call line, plane, or circle (or nothing) object methods
    Eigen::Vector3d desired_position;
#ifdef HYBRID
    if (hybrid_mode_list[LINE_MODE_IDX])
    {

        desired_position = line.GetDesiredCrosstrackLocation(ee_translation, distance_param);
    }
    else if (hybrid_mode_list[PLANE_MODE_IDX])
    {
        desired_position = plane.GetDesiredCrosstrackLocation(ee_translation, distance_param_x, distance_param_y);
    }
    else if (hybrid_mode_list[CIRCLE_MODE_IDX])
    {
        desired_position = circle.GetDesiredCrosstrackLocation(ee_translation, theta_param);
    }
    else if (hybrid_mode_list[SPLINE_MODE_IDX])
    {
        spline.FindDesiredSplinePoint(ee_translation);
        desired_position = spline.GetBestPoint();
    }
#endif

#if !defined(HYBRID)
    // This if-else is for nothing case
    Eigen::Vector3d desired_traj(-0.0001 * (time_in_sec - 10), 0, 0);
    if (time_in_sec < 10)
    {
        Eigen::Vector3d x_offset(0.6, 0, 0);
        desired_position = p_1 + x_offset;
    }
    else
    {
        // desired_position = ee_translation; // Free to move in space
        desired_position = desired_position + desired_traj; // Free to move in space
    }
#endif

    Eigen::Vector3d current_position(ee_translation);
    Eigen::Quaterniond current_orientation(ee_linear);

    // ----------EE Pos: Compute position and orientation error--------------------------------------
    Eigen::Matrix<double, 6, 1> error;
    uint8_t pos_error_flag = 0;
    // Apply a filter to the position error to avoid violent movements
    for (int i = 0; i < 3; i++)
    {
        float error_thresh = 0.02;

        if (desired_position[i] - current_position[i] > error_thresh)
        {
            desired_position[i] = current_position[i] + error_thresh;
            pos_error_flag = 1;
        }
        else if (desired_position[i] - current_position[i] < -error_thresh)
        {
            desired_position[i] = current_position[i] - error_thresh;
            pos_error_flag = 1;
        }
    }
    error.head(3) << desired_position - current_position;

    // -----------EE Twist: Set desired twist in frenet frame----------------------------------------
    Eigen::Matrix<double, 6, 1> desired_velocity_frenet;
    desired_velocity_frenet << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0; // in order e_t,e_b,e_n,e_t,e_b,e_n

    // -----------Compute instantaneous frenet frame and rotation matrix------------------------------
    // 4) Call line, plane, or circle (or nothing) get unit direction methods
    Eigen::Vector3d e_t;
    Eigen::Vector3d e_t_plane;
#ifdef HYBRID
    if (hybrid_mode_list[LINE_MODE_IDX])
    {
        e_t = line.GetLineUnitDirection();
        // std::cout << "e_t vector (line)\n"
        //           << e_t << std::endl;
    }
    else if (hybrid_mode_list[PLANE_MODE_IDX])
    {
        e_t = (plane.GetPlaneUnitDirection()).col(0);
        e_t_plane = (plane.GetPlaneUnitDirection()).col(2);
        // std::cout << "e_t vector (plane)\n"
        //           << e_t << std::endl;
        // std::cout << "e_t_plane vector (plane)\n"
        //           << e_t_plane << std::endl;
    }
    else if (hybrid_mode_list[CIRCLE_MODE_IDX])
    {
        e_t = circle.GetUnitDirection(theta_param);
        // std::cout << "e_t vector (circle)\n"
        //           << e_t << std::endl;
    }
    else if (hybrid_mode_list[SPLINE_MODE_IDX])
    {
        e_t = spline.GetBestTangent();
        // std::cout << "e_t vector (spline)\n"
        //           << e_t << std::endl;
    }
#endif
    Eigen::Vector3d e_n(0, 0, 1);
    if (error.head(3).norm() != 0)
    {
        e_n = -error.head(3) / error.head(3).norm();
    }
    Eigen::Vector3d e_b(e_n.cross(e_t));
    Eigen::Matrix<double, 3, 3> R3_3;
    R3_3 << e_t, e_b, e_n;
    Eigen::Matrix<double, 6, 6> R;
    R.setIdentity();
    R.topLeftCorner(3, 3) << R3_3;
    R.bottomRightCorner(3, 3) << R3_3;

    // std::cout << "R Matrix" << R3_3 << std::endl;

    // Convert current orientation to rotation matrix
    Eigen::Matrix3d cur_R = current_orientation.toRotationMatrix();
    // Convert current rotation matrix to Euler angles (in radians)
    Eigen::Vector3d current_thetas = cur_R.eulerAngles(0, 1, 2);
    std::cout << "current orientation: " << current_thetas << std::endl;

    double theta_x;
    double theta_y;
    double theta_z;

// Calculate desired orientation depending on the current mode
#ifdef HYBRID
    uint8_t e_t_octant = CheckOctant(e_t);
    uint8_t theta_y_flip = (e_t_octant == 0 || e_t_octant == 2 || e_t_octant == 5 || e_t_octant == 7);
    uint8_t theta_z_flip = (e_t_octant == 1 || e_t_octant == 3 || e_t_octant == 5 || e_t_octant == 7);

    uint8_t e_t_plane_octant = CheckOctant(e_t_plane);
    uint8_t theta_x_neg = (e_t_plane_octant == 0 || e_t_plane_octant == 1 || e_t_plane_octant == 4 || e_t_plane_octant == 5);
    uint8_t theta_x_flip = (e_t_plane_octant == 2 || e_t_plane_octant == 3 || e_t_plane_octant == 4 || e_t_plane_octant == 5);

    if (hybrid_mode_list[LINE_MODE_IDX])
    {
        theta_x = M_PI;
        // Calculate the adjusted "adjacent" line length for the theta_y term
        double new_len = sqrt(pow(e_t[0], 2) + pow(e_t[1], 2));
        theta_y = (theta_y_flip ? -1.0 : 1.0) * std::acos(new_len);
        // This term requires a sign flip depending on the unit direction of the defined line
        theta_z = std::atan2((theta_z_flip ? -1.0 : 1.0) * e_t[1], (theta_z_flip ? -1.0 : 1.0) * e_t[0]);
    }
    else if (hybrid_mode_list[PLANE_MODE_IDX])
    {
        // Calculate the adjusted "adjacent" line length for the theta_x term
        double new_len = sqrt(pow(e_t_plane[0], 2) + pow(e_t_plane[1], 2));
        if (theta_x_flip)
        {
            theta_x = (theta_x_neg ? -1.0 : 1.0) * ((M_PI / 2.0) + std::acos(new_len));
        }
        else
        {
            theta_x = (theta_x_neg ? -1.0 : 1.0) * std::asin(new_len);
        }

        theta_y = 0.0;
        // This term requires a sign flip depending on the unit direction of the defined plane
        theta_z = std::atan2(e_t_plane[1], e_t_plane[0]) - (M_PI / 2.0);
    }
    else if (hybrid_mode_list[CIRCLE_MODE_IDX])
    {
        // This is just a test mode, so we don't care about orientation control here
        theta_x = M_PI;
        theta_y = 0.0;
        theta_z = 0.0;
    }
    else if (hybrid_mode_list[SPLINE_MODE_IDX])
    {
        theta_x = M_PI;
        // Calculate the adjusted "adjacent" line length for the theta_y term
        double new_len = sqrt(pow(e_t[0], 2) + pow(e_t[1], 2));
        theta_y = (theta_y_flip ? -1.0 : 1.0) * std::acos(new_len);
        // This term requires a sign flip depending on the unit direction of the defined spline
        theta_z = std::atan2((theta_z_flip ? -1.0 : 1.0) * e_t[1], (theta_z_flip ? -1.0 : 1.0) * e_t[0]);
    }
#endif

    Eigen::Vector3d euler_orientation(theta_x, theta_y, theta_z);
    Eigen::Quaterniond desired_orientation = Eigen::AngleAxisd(euler_orientation[2], Eigen::Vector3d::UnitZ()) *
                                             Eigen::AngleAxisd(euler_orientation[1], Eigen::Vector3d::UnitY()) *
                                             Eigen::AngleAxisd(euler_orientation[0], Eigen::Vector3d::UnitX());

    if (desired_orientation.coeffs().dot(current_orientation.coeffs()) < 0.0)
    {
        current_orientation.coeffs() << -current_orientation.coeffs();
    }
    Eigen::Quaterniond error_quaternion(current_orientation.inverse() * desired_orientation);

    // Convert to angle-axis
    Eigen::AngleAxisd angle_axis(error_quaternion);
    double angle = angle_axis.angle(); // in radians
    Eigen::Vector3d axis = angle_axis.axis();

    // Limit parameters
    double angle_limit = 0.5;

#ifdef HYBRID
    double max_angle = 0.0;
    if (hybrid_mode_list[LINE_MODE_IDX] || hybrid_mode_list[SPLINE_MODE_IDX])
    {
        // Find the max angle axis (ignoring x) for the desired rotation
        max_angle = std::max({abs(axis.y() * angle), abs(axis.z() * angle)});
    }
    else if (hybrid_mode_list[PLANE_MODE_IDX])
    {
        // Find the max angle axis (ignoring z) for the desired rotation
        max_angle = std::max({abs(axis.x() * angle), abs(axis.y() * angle)});
    }
    else
    {
        // Find the max angle axis for the desired rotation
        max_angle = std::max({abs(axis.x() * angle), abs(axis.y() * angle), abs(axis.z() * angle)});
    }
#endif

    double scale_factor = 1.0;
    // Scale the angle if needed
    if (pos_error_flag)
    {
        scale_factor = 0.0;
    }
    else if (max_angle > angle_limit)
    {
        scale_factor = angle_limit / max_angle;
    }
    angle_axis.angle() *= scale_factor;

    // Reconstruct limited rotation quaternion
    Eigen::Quaterniond error_limited(angle_axis);
    error.tail(3) << error_limited.x(), error_limited.y(), error_limited.z();
    error.tail(3) << current_orientation * error.tail(3);
    // error.tail(3) << 0,0,0; //Use this if we don't care about orientation

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
    Eigen::Matrix<double, 6, 6> K_des;
    Eigen::Matrix<double, 6, 6> K_switch;

    K_des.setZero();
    K_des(0, 0) = 500;
    K_des(1, 1) = 500;
    K_des(2, 2) = 500;
    K_des(3, 3) = 30;
    K_des(4, 4) = 30;
    K_des(5, 5) = 30;

    K_switch.setIdentity();
#ifdef HYBRID
    // Set switching matrix elements for position control
    K_switch(0, 0) = 0;
    // Only enable stiffness in the ee y-axis in line, circle, or spline modes
    K_switch(1, 1) = hybrid_mode_list[LINE_MODE_IDX] | hybrid_mode_list[CIRCLE_MODE_IDX] | hybrid_mode_list[SPLINE_MODE_IDX];

    // Set switching matrix elements for orientation control
    // Turn off stiffness in rotation about the ee x-axis in line or spline mode
    K_switch(3, 3) = (!hybrid_mode_list[LINE_MODE_IDX] && !hybrid_mode_list[SPLINE_MODE_IDX]) || pos_error_flag;
    // Turn off stiffness in rotation about the ee z-axis in plane mode
    K_switch(5, 5) = !hybrid_mode_list[PLANE_MODE_IDX] || pos_error_flag;
#endif

    // -----------Compute pseudoinverse for Jacobian transpose-------------------------------------
    Eigen::MatrixXd Jacobian_pseudoInverse_transpose;
    pseudoInverse(J_.data.transpose(), Jacobian_pseudoInverse_transpose, false);

    // -----------Compute null space control law---------------------------------------------------
    // Below we are trying to make the elbow joint to stay at 0 and -pi/2 positions whenever possible
    // Need to revise this later but not important for now
    Eigen::Matrix<double, 7, 1> null_error;
    null_error.setZero();
    null_error(1) = 0.0 - q_.data[1];
    null_error(3) = -1.57 - q_.data[3];
    Eigen::Matrix<double, 7, 1> tau_null;
    Eigen::Matrix<double, 7, 7> identity_7by7;
    identity_7by7.setIdentity();
    tau_null = (identity_7by7 - J_.data.transpose() * Jacobian_pseudoInverse_transpose) * null_error * 300.0; // if the gain is too high the whole thing can vibrate within the null space

    // -----------Compute K_bar (Positive definite K matrix normalized by mass matrix before diagonalized)
    Eigen::Matrix<double, 6, 6> K_bar(L_inverse * R * K_switch * K_des * R.transpose() * L_inverse.transpose());
    // std::cout << "K_bar: " << std::endl << K_bar << std::endl;

    // -----------Compute eigenvalue and eigenvector of K_bar-------------------------------------------
    Eigen::EigenSolver<Eigen::MatrixXd> es(K_bar);
    // std::cout << "The eigenvalues:" << std::endl << es.eigenvalues() << std::endl;
    // std::cout << "The eigenvector:" << std::endl << es.eigenvectors() << std::endl;
    //////Eigen::Matrix<double,6,6> S(es.eigenvectors().real());
    // This eigenvalue is squared root before multiplied by 2 and zeta
    //////Eigen::Matrix<double,6,6> EV(es.eigenvalues().real().array().abs().sqrt().matrix().asDiagonal()*2.0*1.0);
    // std::cout << "EV:" << std::endl << EV << std::endl;
    // std::cout << "S:" << std::endl << S << std::endl;
    //  Select only non zero eigenvales and eigenvectors
    Eigen::Matrix<double, 6, 6> EV;
    EV.setZero();
    Eigen::Matrix<double, 6, 6> S;
    S.setZero();
    int k = 0; // set an index for non zero eigenvalues
    // Loop through to find non zero eigenvalue and asscosiated eigenvector
    for (int i = 0; i < es.eigenvalues().real().size(); ++i)
    {
        if (es.eigenvalues().real()(i) > 0.001)
        {
            EV(k, k) = std::sqrt(es.eigenvalues().real()(i)) * 2.0 * 1.0;
            S.col(k) = es.eigenvectors().real().col(i);
            ++k;
        }
    }
    // std::cout << "EV:" << std::endl << EV_new << std::endl;
    // std::cout << "S:" << std::endl << S_new << std::endl;

    // Compute null space matrix of K_bar (K_bar*v = 0, trying to find )
    Eigen::FullPivLU<Eigen::MatrixXd> lu(K_bar);
    Eigen::MatrixXd null_space = lu.kernel();
    // std::cout << "null_space: " << std::endl << null_space << std::endl;

    // Make the null_space matrix orthogonal
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(null_space);
    Eigen::MatrixXd null_space_orthogonal(qr.householderQ());
    // std::cout << "null_space_orthogonal: " << std::endl << null_space_orthogonal << std::endl;

    // Get a place holder for index for zero eigenvalue and associated eigenvectors
    int null_index = k;

    // Fill in to complete eigenvector matrix S
    for (int i = 0; i < null_space_orthogonal.cols(); ++i)
    {
        if (k > 5)
            break;
        S.col(k) = null_space_orthogonal.col(i);
        ++k;
        if (k > 5)
            break;
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

    // publishing these matrices so they can be recorded to a .bag
    publishMatrices(S, L);

    // ------------Compute desired damping matrix------------------------------------------------------------
    Eigen::Matrix<double, 6, 6> C_des(R.transpose() * L * S * EV * S.transpose() * L.transpose() * R);
    // std::cout << "C_des:" << std::endl << C_des << std::endl;

    // ------------Compute external force in global frame-----------------------------------------------------
    Eigen::Matrix<double, 3, 1> ext_force_global(force_rotation_matrix * ext_force_body);

    //-------------Compute Control Law-------------------------------------------------------------------------
    // Eigen::Matrix<double,4,1> tau_d = J_.data.transpose()*(cartesian_stiffness*error + cartesian_damping*(-J_.data*q_dot_.data)) + G_.data + C_.data;
    Eigen::Matrix<double, 7, 1> tau_d;
    // tau_d = J_.data.transpose()*(R*K_des*R.transpose()*error + R*C_des*R.transpose()*(R*desired_velocity_frenet-J_.data*q_dot_.data)) + G_.data + C_.data;
    double time_begin_in_sec;
    std::cout << "Time now:" << time_in_sec << std::endl;
    if (time_in_sec > 1000.0)
    {
        // if(start_flag){
        //     time_begin_in_sec = time_in_sec;
        //     begin_cartesian_position = desired_position;
        //     start_flag = false;
        // }
        // time_in_sec = time_in_sec - time_begin_in_sec;
        // Eigen::Matrix<double,3,1> desired_position_cartesian;
        // desired_position_cartesian = line.GetDesiredPositionTrajectory(begin_cartesian_position,time_in_sec,5.0);
        // error.head(3) << desired_position_cartesian - current_position;

        // Eigen::Matrix<double,6,1> desired_velocity_cartesian;
        // desired_velocity_cartesian.setZero();
        // desired_velocity_cartesian.head(3) << line.GetDesiredVelocityTrajectory(begin_cartesian_position,time_in_sec,5.0);

        // Eigen::Matrix<double,6,1> desired_acceleration_cartesian;
        // desired_acceleration_cartesian.setZero();
        // desired_acceleration_cartesian.head(3) << line.GetDesiredAccelerationTrajectory(begin_cartesian_position,time_in_sec,5.0);

        // // Control law for trajectory control
        // tau_d = J_.data.transpose()*(mass_cart*desired_acceleration_cartesian+ R*K_des*R.transpose()*error + R*C_des*R.transpose()*(desired_velocity_cartesian-J_.data*q_dot_.data)) + G_.data + C_.data;
    }
    // else if(ext_force_global.dot(e_t) > 50)
    // {
    //     tau_d = J_.data.transpose()*(R*K_des*R.transpose()*error + R*C_des*R.transpose()*(R*desired_velocity_frenet-J_.data*q_dot_.data)) + G_.data + C_.data;
    //     std::cout << "Pos vel" << std::endl;
    // }
    // else if(ext_force_global.dot(e_t) < -50)
    // {
    //     tau_d = J_.data.transpose()*(R*K_des*R.transpose()*error + R*C_des*R.transpose()*(-R*desired_velocity_frenet-J_.data*q_dot_.data)) + G_.data + C_.data;
    //     std::cout << "Neg vel" << std::endl;
    // }
    else
    {
        tau_d = J_.data.transpose() * (R * K_switch * K_des * R.transpose() * error + R * C_des * R.transpose() * (-J_.data * q_dot_.data)) + G_.data + C_.data;
        std::cout << "Cross track only" << std::endl;
    }

    //-------------Compute total control law------------------------------------------------------------------
    tau_d = tau_d + tau_null;

    //-------------For debugging only-------------------------------------------------------------------------
    // std::cout << "C_des:" << std::endl << C_des << std::endl;
    // std::cout << "mass_cart:" << std::endl << mass_cart << std::endl;
    // std::cout << "ST_Kbar_S:" << std::endl << S.transpose()*K_bar*S << std::endl;
    // std::cout << "ST_Kbar_S:" << std::endl << S.transpose()*K_bar*S << std::endl;
    // std::cout << "ST_Cbar_S:" << std::endl << S.transpose()*L_inverse*R*C_des*R.transpose()*L_inverse.transpose()*S << std::endl;
    // std::cout << "ST_Cbar_S:" << std::endl << S.transpose()*L_inverse*R*C_des*R.transpose()*L_inverse.transpose()*S << std::endl;
    // std::cout << "F_ext:" << std::endl << Jacobian_pseudoInverse_transpose*(effort_.data - tau_d + C_.data + G_.data) << std::endl;
    // std::cout << "effort:" << std::endl << effort_.data << std::endl;
    // std::cout << "tau_d-C-G-tau_null:" << std::endl << tau_d - C_.data - G_.data - tau_null << std::endl;
    // std::cout << "tau_d:" << std::endl << tau_d << std::endl;
    // std::cout << "ext_force_body:" << std::endl << ext_force_body << std::endl;
    // std::cout << "ext_force_global" << std::endl << ext_force_global << std::endl;
    std::cout << "ext_force_global_dot_et" << std::endl
              << ext_force_global.dot(e_t) << std::endl;

    // Show R on screen
    /*std::cout << "R:" << std::endl << R << std::endl << std::endl;
    std::cout << current_orientation.w() << std::endl;
    std::cout << current_orientation.x() << std::endl;
    std::cout << current_orientation.y() << std::endl;
    std::cout << current_orientation.z() << std::endl;
    std::cout << "Distance Parameter = " << distance_param << std::endl;
    std::cout << line.GetLineUnitDirection() << std::endl;
    */

    // Publish Control Law to each joint
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

    Eigen::Matrix<double, 6, 1> vel(J_.data * q_dot_.data);
    geometry_msgs::Twist twist;
    twist.linear.x = vel[0];
    twist.linear.y = vel[1];
    twist.linear.z = vel[2];
    twist.angular.x = vel[3];
    twist.angular.y = vel[4];
    twist.angular.z = vel[5];
    twist_pub_.publish(twist);

    // Publish Trajectory
    geometry_msgs::Point traj;
    traj.x = desired_position[0];
    traj.y = desired_position[1];
    traj.z = desired_position[2];
    traj_pub_.publish(traj);

    // Publish Spline marker
    spline_pub.publish(spline.GetSplineVis());
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
    apply_wrench_req.wrench.force.x = ext_force_joystick[0] * max_force / 4;
    apply_wrench_req.wrench.force.y = ext_force_joystick[1] * max_force / 4;
    apply_wrench_req.wrench.force.z = ext_force_joystick[2] * max_force / 4;
    // apply_wrench_req.start_time = 0;
    apply_wrench_req.duration = (ros::Duration)(-1);
    wrench_client.call(apply_wrench_req, apply_wrench_resp);

    // // This part is to move joint4 out of awkward position
    // if (buttonR2 && buttonX)
    // {
    //     std_msgs::Float64 msg;
    //     msg.data = 100;
    //     joint4_torque_pub_.publish(msg);
    // }
    // else if (buttonX)
    // {
    //     std_msgs::Float64 msg;
    //     msg.data = -100;
    //     joint4_torque_pub_.publish(msg);
    // }

    // Handle hybrid mode switching
    if (buttonA)
    {
        // LINE
        desired_mode_idx = 0;
    }
    else if (buttonB)
    {
        // PLANE
        desired_mode_idx = 1;
    }
    else if (buttonY)
    {
        // CIRCLE
        desired_mode_idx = 2;
    }
    else if (buttonX)
    {
        // SPLINE
        desired_mode_idx = 3;
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

void ForceSensorPublisher(const geometry_msgs::WrenchStamped::ConstPtr &forceSensorPtr_)
{
    ext_force_body[0] = forceSensorPtr_->wrench.force.x;
    ext_force_body[1] = forceSensorPtr_->wrench.force.y;
    ext_force_body[2] = forceSensorPtr_->wrench.force.z;
}

// Function to convert Eigen matrix to Float64MultiArray and publish
void publishMatrices(const Eigen::MatrixXd &S, const Eigen::MatrixXd &L)
{
    std_msgs::Float64MultiArray msg;

    // Flatten the first matrix (S) into the message
    for (int i = 0; i < S.rows(); ++i)
    {
        for (int j = 0; j < S.cols(); ++j)
        {
            msg.data.push_back(S(i, j)); // Add matrix element to the message
        }
    }

    // Flatten the second matrix (L) into the message
    for (int i = 0; i < L.rows(); ++i)
    {
        for (int j = 0; j < L.cols(); ++j)
        {
            msg.data.push_back(L(i, j)); // Add matrix element to the message
        }
    }

    // Publish the message
    matrix_pub.publish(msg);
}

// Function to return the octant of a 3d vector
uint8_t CheckOctant(Eigen::Vector3d e_t)
{
    uint8_t e_t_octant = 0;
    // (+,+,+)
    if (e_t[0] > 0 && e_t[1] > 0 && e_t[2] > 0)
    {
        e_t_octant = 0;
    }
    // (+,-,+)
    else if (e_t[0] > 0 && e_t[1] < 0 && e_t[2] > 0)
    {
        e_t_octant = 1;
    }
    // (-,+,+)
    else if (e_t[0] < 0 && e_t[1] > 0 && e_t[2] > 0)
    {
        e_t_octant = 2;
    }
    // (-,-,+)
    else if (e_t[0] < 0 && e_t[1] < 0 && e_t[2] > 0)
    {
        e_t_octant = 3;
    }
    // (+,+,-)
    else if (e_t[0] > 0 && e_t[1] > 0 && e_t[2] < 0)
    {
        e_t_octant = 4;
    }
    // (+,-,-)
    else if (e_t[0] > 0 && e_t[1] < 0 && e_t[2] < 0)
    {
        e_t_octant = 5;
    }
    // (-,+,-)
    else if (e_t[0] < 0 && e_t[1] > 0 && e_t[2] < 0)
    {
        e_t_octant = 6;
    }
    // (-,-,-)
    else if (e_t[0] < 0 && e_t[1] < 0 && e_t[2] < 0)
    {
        e_t_octant = 7;
    }

    return e_t_octant;
}