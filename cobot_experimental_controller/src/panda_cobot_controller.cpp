// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cobot_experimental_controller/panda_cobot_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <algorithm>
#include <iostream>

#include <space_manipulation/circle3d.h>
#include <space_manipulation/line3d.h>
#include <space_manipulation/plane3d.h>
#include "cobot_experimental_controller/pseudo_inversion.h"
#include "space_manipulation/spline3d.h"
#include "std_msgs/UInt8MultiArray.h"

#include <sensor_msgs/Joy.h>
#include <Eigen/Geometry>

namespace cobot_experimental_controller
{

  // Points for line and plane
  Eigen::Vector3d p_1(0.5, -0.25, 0.15);
  Eigen::Vector3d p_2(0.4, 0.25, 0.3);
  Eigen::Vector3d p_3(0.55, 0.0, 0.25);
  // Points for circle
  Eigen::Vector3d p_c1(0.38, 0.25, 0.5);
  Eigen::Vector3d p_c2(0.35, 0.0, 0.75);
  Eigen::Vector3d p_cc(0.4, 0.0, 0.5);
  // Points for spline
  // You can define any number of points here
  std::vector<Eigen::Vector3d> spline_points = {
      Eigen::Vector3d(0.6, -0.5, 0.1),
      Eigen::Vector3d(0.5, -0.25, 0.25),
      Eigen::Vector3d(0.6, 0.2, 0.3),
      Eigen::Vector3d(0.5, 0.5, 0.5),
      Eigen::Vector3d(0.6, 0.75, 0.6)};

// 2) Choose line or plane or circle objects
#ifdef HYBRID
  // Setup line, plane, and circle parameters
  Line3d line(p_1, p_2);
  double distance_param;
  Plane3d plane(p_1, p_2, p_3);
  double distance_param_x, distance_param_y;
  Circle3d circle(p_c1, p_c2, p_cc);
  double theta_param;
  Spline3d spline(spline_points);
#endif

  bool PandaCobotController::init(hardware_interface::RobotHW *robot_hw,
                                  ros::NodeHandle &node_handle)
  {
    std::vector<double> cartesian_stiffness_vector;
    std::vector<double> cartesian_damping_vector;
    pose_pub_ = node_handle.advertise<geometry_msgs::Pose>("/pose", 1);
    traj_pub_ = node_handle.advertise<geometry_msgs::Pose>("/traj", 1);
    fext_ee_pub_ = node_handle.advertise<geometry_msgs::Vector3>("/f_ext", 1);
    cur_pos_ee_pub_ = node_handle.advertise<geometry_msgs::Vector3>("/cur_pos_ee", 1);
    des_pos_ee_pub_ = node_handle.advertise<geometry_msgs::Vector3>("/des_pos_ee", 1);
    k_switch_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("/k_switch", 1);
    hybrid_mode_pub_ = node_handle.advertise<std_msgs::UInt8MultiArray>("/hybrid_mode", 1);
    spline_pub = node_handle.advertise<visualization_msgs::Marker>("/spline", 1);

    // Set rosparameters for line
    node_handle.setParam("/p_initial_x", p_1[0]);
    node_handle.setParam("/p_initial_y", p_1[1]);
    node_handle.setParam("/p_initial_z", p_1[2]);
    node_handle.setParam("/p_final_x", p_2[0]);
    node_handle.setParam("/p_final_y", p_2[1]);
    node_handle.setParam("/p_final_z", p_2[2]);
    node_handle.setParam("/p_plane_x", p_3[0]);
    node_handle.setParam("/p_plane_y", p_3[1]);
    node_handle.setParam("/p_plane_z", p_3[2]);
    // Must flatten the spline points matrix so it can be set as a rosparameter
    std::vector<double> spline_points_flat;
    for (int i = 0; i < spline_points.size(); i++)
    {
      for (int j = 0; j < 3; j++)
      {
        spline_points_flat.push_back(spline_points[i][j]);
      }
    }
    node_handle.setParam("/spline_points", spline_points_flat);
    // Set rosparameters for plane
    Plane3d plane(p_1, p_2, p_3);
    Eigen::Quaterniond plane_orientation(plane.GetPlaneUnitDirection());
    node_handle.setParam("/plane_orientation_w", plane_orientation.w());
    node_handle.setParam("/plane_orientation_x", plane_orientation.x());
    node_handle.setParam("/plane_orientation_y", plane_orientation.y());
    node_handle.setParam("/plane_orientation_z", plane_orientation.z());
    // Set rosparameters for circle
    node_handle.setParam("/p_center_x", p_cc[0]);
    node_handle.setParam("/p_center_y", p_cc[1]);
    node_handle.setParam("/p_center_z", p_cc[2]);
    Circle3d circle(p_c1, p_c2, p_cc);
    Eigen::MatrixXd R_circle = circle.GetRot();
    // Must flatten the R matrix so it can be set as a rosparameter
    std::vector<double> R_flat = {R_circle(0, 0), R_circle(0, 1), R_circle(0, 2),
                                  R_circle(1, 0), R_circle(1, 1), R_circle(1, 2),
                                  R_circle(2, 0), R_circle(2, 1), R_circle(2, 2)};
    node_handle.setParam("/circle_rad", circle.GetRadius());
    node_handle.setParam("/circle_rot", R_flat);

    sub_equilibrium_pose_ =
        node_handle.subscribe("/equilibrium_pose", 20, &PandaCobotController::equilibriumPoseCallback,
                              this, ros::TransportHints().reliable().tcpNoDelay());

    joystickSubscriber = node_handle.subscribe("/joy", 1, &PandaCobotController::JoystickFeedback,
                                               this, ros::TransportHints().reliable().tcpNoDelay());

    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR_STREAM("PandaCobotController: Could not read parameter arm_id");
      return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
    {
      ROS_ERROR(
          "PandaCobotController: Invalid or no joint_names parameters provided, "
          "aborting controller init!");
      return false;
    }

    auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr)
    {
      ROS_ERROR_STREAM("PandaCobotController: Error getting model interface from hardware");
      return false;
    }
    try
    {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
          model_interface->getHandle(arm_id + "_model"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "PandaCobotController: Exception getting model handle from interface: " << ex.what());
      return false;
    }

    auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR_STREAM("PandaCobotController: Error getting state interface from hardware");
      return false;
    }
    try
    {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
          state_interface->getHandle(arm_id + "_robot"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "PandaCobotController: Exception getting state handle from interface: " << ex.what());
      return false;
    }

    auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM("PandaCobotController: Error getting effort joint interface from hardware");
      return false;
    }
    for (size_t i = 0; i < 7; ++i)
    {
      try
      {
        joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException &ex)
      {
        ROS_ERROR_STREAM("PandaCobotController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    position_d_.setZero();
    orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
    position_d_target_.setZero();
    orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

    cartesian_stiffness_.setZero();
    cartesian_damping_.setZero();

    return true;
  }

  void PandaCobotController::starting(const ros::Time & /*time*/)
  {
    // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
    // to initial configuration
    franka::RobotState initial_state = state_handle_->getRobotState();
    // get jacobian
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    // convert to eigen
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_initial(initial_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

    // set equilibrium point to current state
    position_d_ = initial_transform.translation();
    orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
    position_d_target_ = initial_transform.translation();
    orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

    // set nullspace equilibrium configuration to initial q
    q_d_nullspace_ = q_initial;
  }

  void PandaCobotController::update(const ros::Time &time, const ros::Duration &period)
  {
    // Get robot's state and computed model
    franka::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    std::array<double, 49> mass_joint_array = model_handle_->getMass();
    // Get time now
    double time_in_sec = time.now().toSec();

    // Estimate the external force on the end effector
    std::array<double, 6> external_wrench = robot_state.O_F_ext_hat_K;

    // convert to Eigen
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 7>> mass_joint(mass_joint_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d( // NOLINT (readability-identifier-naming)
        robot_state.tau_J_d.data());
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d ee_translation(transform.translation());
    Eigen::Quaterniond ee_linear(transform.linear());

    // compute mass in Cartesian space
    Eigen::MatrixXd mass_joint_invert;
    pseudoInverse(mass_joint, mass_joint_invert, false);
    Eigen::MatrixXd mass_cart;
    pseudoInverse(jacobian * mass_joint_invert * jacobian.transpose(), mass_cart, false);

    // Compute Cholesky Decomposition
    Eigen::LLT<Eigen::MatrixXd> lltofmass_cart(mass_cart);
    Eigen::MatrixXd L = lltofmass_cart.matrixL();
    Eigen::MatrixXd L_inverse;
    pseudoInverse(L, L_inverse, false);

    // -EE Pos: Set Desired and get current ee position and orientation--
    // Call line or plane (or nothing) object methods
    Eigen::Vector3d desired_position;
#ifdef HYBRID
    if (hybrid_mode_list[LINE_MODE_IDX])
    {
      desired_position = line.GetDesiredCrosstrackLocation(ee_translation, distance_param);
    }
    else if (hybrid_mode_list[PLANE_MODE_IDX])
    {
      desired_position =
          plane.GetDesiredCrosstrackLocation(ee_translation, distance_param_x, distance_param_y);
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

    // ----------EE Pos: Compute position and orientation error-----------
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

    // -----------EE Twist: Set desired twist in frenet frame-------------
    Eigen::Matrix<double, 6, 1> desired_velocity_frenet;
    desired_velocity_frenet << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0; // in order e_t,e_b,e_n,e_t,e_b,e_n

    // -------Compute instantaneous frenet frame and rotation matrix------
    // Call line or plane (or nothing) get unit direction methods
    Eigen::Vector3d e_t;
    Eigen::Vector3d e_t_plane;
#ifdef HYBRID
    if (hybrid_mode_list[LINE_MODE_IDX])
    {
      e_t = line.GetLineUnitDirection();
    }
    else if (hybrid_mode_list[PLANE_MODE_IDX])
    {
      e_t = (plane.GetPlaneUnitDirection()).col(0);
      e_t_plane = (plane.GetPlaneUnitDirection()).col(2);
    }
    else if (hybrid_mode_list[CIRCLE_MODE_IDX])
    {
      e_t = circle.GetUnitDirection(theta_param);
    }
    else if (hybrid_mode_list[SPLINE_MODE_IDX])
    {
      e_t = spline.GetBestTangent();
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

    // Convert current orientation to rotation matrix
    Eigen::Matrix3d cur_R = current_orientation.toRotationMatrix();
    // Convert current rotation matrix to Euler angles (in radians)
    Eigen::Vector3d current_thetas = cur_R.eulerAngles(0, 1, 2);

    double theta_x;
    double theta_y;
    double theta_z;

// Calculate desired orientation depending on the current mode
#ifdef HYBRID
    uint8_t e_t_octant = CheckOctant(e_t);
    uint8_t theta_y_flip = (e_t_octant == 0 || e_t_octant == 2 || e_t_octant == 5 || e_t_octant == 7);
    uint8_t theta_z_flip = (e_t_octant == 1 || e_t_octant == 3 || e_t_octant == 5 || e_t_octant == 7);

    uint8_t e_t_plane_octant = CheckOctant(e_t_plane);
    uint8_t theta_x_neg = (e_t_plane_octant == 0 || e_t_plane_octant == 1 || e_t_plane_octant == 4 ||
                           e_t_plane_octant == 5);
    uint8_t theta_x_flip = (e_t_plane_octant == 2 || e_t_plane_octant == 3 || e_t_plane_octant == 4 ||
                            e_t_plane_octant == 5);

    if (hybrid_mode_list[LINE_MODE_IDX])
    {
      theta_x = M_PI;
      // Calculate the adjusted "adjacent" line length for the theta_y term
      double new_len = sqrt(pow(e_t[0], 2) + pow(e_t[1], 2));
      theta_y = (theta_y_flip ? -1.0 : 1.0) * std::acos(new_len);
      // This term requires a sign flip depending on the unit direction of the defined line
      theta_z =
          std::atan2((theta_z_flip ? -1.0 : 1.0) * e_t[1], (theta_z_flip ? -1.0 : 1.0) * e_t[0]);
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
      theta_z =
          std::atan2((theta_z_flip ? -1.0 : 1.0) * e_t[1], (theta_z_flip ? -1.0 : 1.0) * e_t[0]);
    }
#endif

    Eigen::Vector3d euler_orientation(theta_x, theta_y, theta_z);
    Eigen::Quaterniond desired_orientation =
        Eigen::AngleAxisd(euler_orientation[2], Eigen::Vector3d::UnitZ()) *
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
    double angle_limit = 0.1;

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
    Eigen::Quaterniond q_desired = current_orientation * error_limited;
    error.tail(3) << error_limited.x(), error_limited.y(), error_limited.z();
    error.tail(3) << current_orientation * error.tail(3);
    // error.tail(3) << 0,0,0; //Use this if we don't care about orientation

    // -----------Specify cartesian stiffness-----------------------------
    Eigen::Matrix<double, 6, 6> K_des;
    Eigen::Matrix<double, 6, 6> K_switch;

    K_des.setZero();
    K_des(0, 0) = 600;
    K_des(1, 1) = 600;
    K_des(2, 2) = 600;
    K_des(3, 3) = 60;
    K_des(4, 4) = 60;
    K_des(5, 5) = 60;

    K_switch.setIdentity();
#ifdef HYBRID
    // Set switching matrix elements for position control
    K_switch(0, 0) = 0;
    // Only enable stiffness in the ee y-axis in line, circle, or spline modes
    K_switch(1, 1) = hybrid_mode_list[LINE_MODE_IDX] || hybrid_mode_list[CIRCLE_MODE_IDX] ||
                     hybrid_mode_list[SPLINE_MODE_IDX];

    // Set switching matrix elements for orientation control
    // Turn off stiffness in rotation about the ee x-axis in line or spline mode
    K_switch(3, 3) =
        (!hybrid_mode_list[LINE_MODE_IDX] && !hybrid_mode_list[SPLINE_MODE_IDX]) || pos_error_flag;
    // Turn off stiffness in rotation about the ee z-axis in plane mode
    K_switch(5, 5) = !hybrid_mode_list[PLANE_MODE_IDX] || pos_error_flag;
#endif

    // -----------Compute pseudoinverse for Jacobian transpose------------
    Eigen::MatrixXd Jacobian_pseudoInverse_transpose;
    pseudoInverse(jacobian.transpose(), Jacobian_pseudoInverse_transpose, false);

    // -----------Compute null space control law--------------------------
    // Below we are trying to make the elbow joint to stay at 0 and -pi/2 positions whenever possible
    // Need to revise this later but not important for now
    Eigen::Matrix<double, 7, 1> null_error;
    null_error.setZero();
    null_error(1) = 0.0 - current_position[1];
    null_error(3) = -1.57 - current_position[3];
    Eigen::Matrix<double, 7, 1> tau_null;
    Eigen::Matrix<double, 7, 7> identity_7by7;
    identity_7by7.setIdentity();
    tau_null = (identity_7by7 - jacobian.transpose() * Jacobian_pseudoInverse_transpose) *
               null_error *
               10.0; // if the gain is too high the whole thing can vibrate within the null space

    // Compute K_bar (Positive definite K matrix normalized by mass matrix before
    // diagonalized)
    Eigen::Matrix<double, 6, 6> K_bar(L_inverse * R * K_switch * K_des * R.transpose() *
                                      L_inverse.transpose());

    // -----------Compute eigenvalue and eigenvector of K_bar-------------
    Eigen::EigenSolver<Eigen::MatrixXd> es(K_bar);
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

    // Compute null space matrix of K_bar (K_bar*v = 0, trying to find )
    Eigen::FullPivLU<Eigen::MatrixXd> lu(K_bar);
    Eigen::MatrixXd null_space = lu.kernel();

    // Make the null_space matrix orthogonal
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(null_space);
    Eigen::MatrixXd null_space_orthogonal(qr.householderQ());

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

    // ------------Compute desired damping
    // matrix------------------------------------------------------------
    Eigen::Matrix<double, 6, 6> C_des(R.transpose() * L * S * EV * S.transpose() * L.transpose() * R);

    //-------------Compute ControlLaw-------------------------------------
    // Gravity term is not necessary because the hardware driver handles the weight of the panda
    Eigen::Matrix<double, 7, 1> tau_d;
    double time_begin_in_sec;
    tau_d = jacobian.transpose() * (R * K_switch * K_des * R.transpose() * error +
                                    R * C_des * R.transpose() * (-jacobian * dq)) +
            coriolis;

    //-------------Compute total control law------------------------------
    tau_d = tau_d; // + tau_null;

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

    // Publish Trajectory
    geometry_msgs::Pose traj;
    traj.position.x = desired_position[0];
    traj.position.y = desired_position[1];
    traj.position.z = desired_position[2];
    traj.orientation.x = q_desired.x();
    traj.orientation.y = q_desired.y();
    traj.orientation.z = q_desired.z();
    traj.orientation.w = q_desired.w();
    traj_pub_.publish(traj);

    // Publish Spline marker
    spline_pub.publish(spline.GetSplineVis());

    // Publish a bunch of stuff to visualize results

    // Calculate the Homogeneous Transformation Matrix of the EE frame with respect to the global
    // frame
    Eigen::Vector3d p_origin_ee = -R3_3 * current_position;
    Eigen::Matrix<double, 4, 4> T;
    T.setIdentity();
    T.block<3, 3>(0, 0) = R3_3;
    T.block<3, 1>(0, 3) = p_origin_ee;

    // Global Frame & EE current and desired position in the EE frame
    Eigen::Vector4d current_position_homogeneous;
    current_position_homogeneous << current_position[0], current_position[1], current_position[2],
        1.0;
    Eigen::Vector4d current_position_ee_homogeneous = T * current_position_homogeneous;
    Eigen::Vector3d current_position_ee = current_position_ee_homogeneous.head<3>();
    // std::cout << "ee position: " << current_position_ee << std::endl;
    geometry_msgs::Vector3 current_position_ee_final;
    current_position_ee_final.x = current_position_ee.x();
    current_position_ee_final.y = current_position_ee.y();
    current_position_ee_final.z = current_position_ee.z();
    cur_pos_ee_pub_.publish(current_position_ee_final);

    Eigen::Vector4d desired_position_homogeneous;
    desired_position_homogeneous << desired_position[0], desired_position[1], desired_position[2],
        1.0;
    Eigen::Vector4d desired_position_ee_homogeneous = T * desired_position_homogeneous;
    Eigen::Vector3d desired_position_ee = desired_position_ee_homogeneous.head<3>();
    // std::cout << "ee des position: " << desired_position_ee << std::endl;
    geometry_msgs::Vector3 desired_position_ee_final;
    desired_position_ee_final.x = desired_position_ee.x();
    desired_position_ee_final.y = desired_position_ee.y();
    desired_position_ee_final.z = desired_position_ee.z();
    des_pos_ee_pub_.publish(desired_position_ee_final);

    // Fext in the EE frame
    Eigen::Vector4d Fext_homogeneous;
    Fext_homogeneous << external_wrench[0], external_wrench[1], external_wrench[2], 1.0;
    Eigen::Vector4d Fext_ee_homogeneous = T * Fext_homogeneous;
    Eigen::Vector3d Fext_ee = Fext_ee_homogeneous.head<3>();
    // std::cout << "ee position: " << current_position_ee << std::endl;
    geometry_msgs::Vector3 Fext_ee_final;
    Fext_ee_final.x = Fext_ee.x();
    Fext_ee_final.y = Fext_ee.y();
    Fext_ee_final.z = Fext_ee.z();
    fext_ee_pub_.publish(Fext_ee_final);

    // K_switch matrix
    std_msgs::Float64MultiArray k_switch_final;
    k_switch_final.layout.dim.resize(2);
    k_switch_final.layout.dim[0].label = "rows";
    k_switch_final.layout.dim[0].size = 6;
    k_switch_final.layout.dim[0].stride = 36; // 6*6
    k_switch_final.layout.dim[1].label = "cols";
    k_switch_final.layout.dim[1].size = 6;
    k_switch_final.layout.dim[1].stride = 6;
    k_switch_final.data.resize(36);
    // Fill data in row-major order
    for (int i = 0; i < 6; ++i)
    {
      for (int j = 0; j < 6; ++j)
      {
        k_switch_final.data[i * 6 + j] = K_switch(i, j);
      }
    }
    k_switch_pub_.publish(k_switch_final);

    // Always publish the current hybrid mode
    std_msgs::UInt8MultiArray hybrid_mode;
    hybrid_mode.data = hybrid_mode_list;
    hybrid_mode_pub_.publish(hybrid_mode);

    // Saturate torque rate to avoid discontinuities
    tau_d << saturateTorqueRate(tau_d, tau_J_d);
    for (size_t i = 0; i < 7; ++i)
    {
      joint_handles_[i].setCommand(tau_d(i));
    }
  }

  Eigen::Matrix<double, 7, 1> PandaCobotController::saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
      const Eigen::Matrix<double, 7, 1> &tau_J_d)
  { // NOLINT (readability-identifier-naming)
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++)
    {
      double difference = tau_d_calculated[i] - tau_J_d[i];
      tau_d_saturated[i] =
          tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
    }
    return tau_d_saturated;
  }

  void PandaCobotController::equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
    orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
        msg->pose.orientation.z, msg->pose.orientation.w;
    if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0)
    {
      orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
    }
  }

  // Function to handle user input from the remote controller
  void PandaCobotController::JoystickFeedback(const sensor_msgs::Joy::ConstPtr &joystickhandlePtr_)
  {
    ext_force_joystick[0] = (joystickhandlePtr_->axes[0]) *
                            (-1.0); // force along x-axis. Left analog controller going left/right
    ext_force_joystick[1] = (joystickhandlePtr_->axes[2]) *
                            (-1.0); // force along y-axis. Right analog controller going left/right
    ext_force_joystick[2] =
        joystickhandlePtr_->axes[1];                // force along z-axis. Left analog controller going up/down
    bool buttonX = joystickhandlePtr_->buttons[0];  // X button
    bool buttonA = joystickhandlePtr_->buttons[1];  // A button
    bool buttonB = joystickhandlePtr_->buttons[2];  // B button
    bool buttonY = joystickhandlePtr_->buttons[3];  // Y button
    bool buttonR2 = joystickhandlePtr_->buttons[7]; // R2 button

    // Handle hybrid mode switching
    if (buttonA)
    {
      // ROS_INFO("button A pressed");
      // LINE
      desired_mode_idx = LINE_MODE_IDX;
    }
    else if (buttonB)
    {
      // ROS_INFO("button B pressed");
      // PLANE
      desired_mode_idx = PLANE_MODE_IDX;
    }
    else if (buttonY)
    {
      // ROS_INFO("button Y pressed");
      // CIRCLE
      desired_mode_idx = CIRCLE_MODE_IDX;
    }
    else if (buttonX)
    {
      // ROS_INFO("button X pressed");
      // SPLINE
      desired_mode_idx = SPLINE_MODE_IDX;
    }

    // Update they hybrid mode list based on button inputs
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
  }

  // Function to check the octant of a 3D vector
  uint8_t PandaCobotController::CheckOctant(Eigen::Vector3d e_t)
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

} // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(cobot_experimental_controller::PandaCobotController,
                       controller_interface::ControllerBase)