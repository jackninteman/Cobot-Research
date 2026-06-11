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
#include <array>
#include <sstream>
#include <string>

#include <space_manipulation/circle3d.h>
#include <space_manipulation/line3d.h>
#include <space_manipulation/plane2d.h>
#include "cobot_experimental_controller/pseudo_inversion.h"
#include "space_manipulation/spline3d.h"
#include "space_manipulation/plane3d.h"

#include <sensor_msgs/Joy.h>
#include <Eigen/Geometry>

namespace cobot_experimental_controller
{

  // Points for line and plane
  std::vector<Eigen::Vector3d> line_plane_points;
  bool line_plane_init = false;
  bool line_plane_class_init = false;
  // Points for circle
  std::vector<Eigen::Vector3d> circle_points;
  bool circle_init = false;
  bool circle_class_init = false;
  // Points for spline
  std::vector<Eigen::Vector3d> spline_points;
  bool spline_init = false;
  bool spline_class_init = false;
  // Points for simple 3d plane
  Eigen::Matrix<double, 4, 4> T_plane_3d;
  std::vector<Eigen::Vector3d> plane_3d_points;
  bool plane_3d_init = false;
  bool plane_3d_class_init = false;

// 2) Choose line or plane or circle objects
#ifdef HYBRID
  // Declare line, plane, circle, and spline parameters
  std::shared_ptr<Line3d> line;
  double distance_param;
  std::shared_ptr<Plane2d> plane_2d;
  double distance_param_x, distance_param_y;
  std::shared_ptr<Circle3d> circle;
  double theta_param;
  std::shared_ptr<Spline3d> spline;
  std::shared_ptr<Plane3d> plane_3d;
#endif

  bool PandaCobotController::init(hardware_interface::RobotHW *robot_hw,
                                  ros::NodeHandle &node_handle)
  {
    std::vector<double> default_torques = {25.0, 25.0, 23.0, 23.0, 20.0, 17.5, 15.0};
    std::vector<double> default_forces = {40.0, 40.0, 40.0, 40.0, 40.0, 40.0};

    // Read torque thresholds
    lower_torque_acc = readCollisionThreshold(node_handle,
                                              "lower_torque_thresholds_acceleration", default_torques);
    upper_torque_acc = readCollisionThreshold(node_handle,
                                              "upper_torque_thresholds_acceleration", default_torques);
    lower_torque_nom = readCollisionThreshold(node_handle,
                                              "lower_torque_thresholds_nominal", default_torques);
    upper_torque_nom = readCollisionThreshold(node_handle,
                                              "upper_torque_thresholds_nominal", default_torques);

    // Read force thresholds
    lower_force_acc = readCollisionThreshold(node_handle,
                                             "lower_force_thresholds_acceleration", default_forces);
    upper_force_acc = readCollisionThreshold(node_handle,
                                             "upper_force_thresholds_acceleration", default_forces);
    lower_force_nom = readCollisionThreshold(node_handle,
                                             "lower_force_thresholds_nominal", default_forces);
    upper_force_nom = readCollisionThreshold(node_handle,
                                             "upper_force_thresholds_nominal", default_forces);

    torque_lower_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("/torque_lower", 1);
    torque_upper_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("/torque_upper", 1);
    force_lower_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("/force_lower", 1);
    force_upper_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("/force_upper", 1);

    std::vector<double> cartesian_stiffness_vector;
    std::vector<double> cartesian_damping_vector;
    pose_pub_ = node_handle.advertise<geometry_msgs::Pose>("/pose", 1);
    traj_pub_ = node_handle.advertise<geometry_msgs::Pose>("/traj", 1);
    fext_ee_pub_ = node_handle.advertise<geometry_msgs::Vector3>("/f_ext", 1);
    tauext_ee_pub_ = node_handle.advertise<geometry_msgs::Vector3>("/tau_ext", 1);
    cur_pos_ee_pub_ = node_handle.advertise<geometry_msgs::Vector3>("/cur_pos_ee", 1);
    des_pos_ee_pub_ = node_handle.advertise<geometry_msgs::Vector3>("/des_pos_ee", 1);
    k_switch_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("/k_switch", 1);
    tau_d_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("/tau_d", 1);
    tau_hat_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("/tau_hat", 1);
    time_now_pub_ = node_handle.advertise<std_msgs::Float64>("/time_now", 1);
    joint_collision_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("/joint_collision", 1);
    cart_collision_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("/cart_collision", 1);
    rot_error_vec_pub_ = node_handle.advertise<geometry_msgs::Vector3>("/rot_error_vec", 1);
    x_dot_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("/x_dot", 1);

    sub_equilibrium_pose_ =
        node_handle.subscribe("/equilibrium_pose", 20, &PandaCobotController::equilibriumPoseCallback,
                              this, ros::TransportHints().reliable().tcpNoDelay());

    // Setup subscriber to hybrid_mode
    hybrid_mode_sub = node_handle.subscribe("/hybrid_mode", 10, &PandaCobotController::hybridModeCallback, this, ros::TransportHints().reliable().tcpNoDelay());

    // Setup subscriber to orientation_mode
    orientation_mode_sub = node_handle.subscribe("/orientation_mode", 10, &PandaCobotController::orientationModeCallback, this, ros::TransportHints().reliable().tcpNoDelay());

    // Setup sebscriber to orientation_mode
    control_mode_sub = node_handle.subscribe("/control_mode", 10, &PandaCobotController::controlModeCallback, this, ros::TransportHints().reliable().tcpNoDelay());

    // Setup subscriber to line_plane_points
    line_plane_sub = node_handle.subscribe("/line_plane_points", 10, &PandaCobotController::linePlaneCallback, this, ros::TransportHints().reliable().tcpNoDelay());

    // Setup subscriber to circle_points
    circle_sub = node_handle.subscribe("/circle_points", 10, &PandaCobotController::circleCallback, this, ros::TransportHints().reliable().tcpNoDelay());

    // Setup subscriber to spline_points
    spline_sub = node_handle.subscribe("/spline_points", 10, &PandaCobotController::splineCallback, this, ros::TransportHints().reliable().tcpNoDelay());

    // Setup subscriber to simple 3d plane points
    plane_3d_points_sub = node_handle.subscribe("/plane_3d_points", 10, &PandaCobotController::plane3dCallback, this, ros::TransportHints().reliable().tcpNoDelay());

    // Setup subscriber to simple 3d plane T matrix
    plane_3d_T_sub = node_handle.subscribe("/plane_3d_T", 10, &PandaCobotController::plane3dTCallback, this, ros::TransportHints().reliable().tcpNoDelay());

    mode_switch_flag_sub = node_handle.subscribe("/mode_switch_flag", 10, &PandaCobotController::modeSwitchFlagCallback, this, ros::TransportHints().reliable().tcpNoDelay());

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
    // First, check to see if geometries have been published. If they have not, we need to command the robot to remain still
    if (line_plane_init && circle_init && spline_init && plane_3d_init)
    {
      // If geometries have been published, we only need to init the class instances once because they shapes are constant (for now)
      if (!line_plane_class_init)
      {
        time_begin_in_sec = time.now().toSec();
        ROS_INFO_STREAM("line & plane was init\n\n");
        line = std::make_shared<Line3d>(line_plane_points[0], line_plane_points[1]);
        plane_2d = std::make_shared<Plane2d>(line_plane_points[0], line_plane_points[1], line_plane_points[2]);
        line_plane_class_init = true;
      }
      if (!circle_class_init)
      {
        ROS_INFO_STREAM("circle was init\n\n");
        circle = std::make_shared<Circle3d>(circle_points[0], circle_points[1], circle_points[2]);
        circle_class_init = true;
      }
      if (!spline_class_init)
      {
        ROS_INFO_STREAM("spline was init\n\n");
        spline = std::make_shared<Spline3d>(spline_points);
        spline_class_init = true;
      }
      if (!plane_3d_class_init)
      {
        ROS_INFO_STREAM("simple 3d plane was init\n\n");
        plane_3d = std::make_shared<Plane3d>(plane_3d_points);
        plane_3d_class_init = true;
      }
    }

    // Get robot's state and computed model
    franka::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    std::array<double, 49> mass_joint_array = model_handle_->getMass();

    // Get time now
    double time_in_sec = time.now().toSec() - time_begin_in_sec;
    std_msgs::Float64 time_now;
    time_now.data = time_in_sec;
    time_now_pub_.publish(time_now);

    // Estimate the external force on the end effector
    std::vector<double> external_wrench(robot_state.K_F_ext_hat_K.begin(), robot_state.K_F_ext_hat_K.end());
    // Estimate the external torque on the end effector
    std::vector<double> external_torque(robot_state.tau_ext_hat_filtered.begin(), robot_state.tau_ext_hat_filtered.end());
    // std::array<double, 6> external_wrench = robot_state.O_F_ext_hat_K;
    // Check which joint contact thresholds are active
    std::array<double, 7> joint_contact = robot_state.joint_contact;
    // Check which joint collision thresholds are active
    std::array<double, 7> joint_collision = robot_state.joint_collision;
    // Check which cartesian contact thresholds are active
    std::array<double, 6> cart_contact = robot_state.cartesian_contact;
    // Check which cartesian collision thresholds are exceeded
    std::array<double, 6> cart_collision = robot_state.cartesian_collision;

    std_msgs::Float64MultiArray joint_collision_msg;
    joint_collision_msg.data.clear();
    joint_collision_msg.data.insert(joint_collision_msg.data.end(), joint_collision.begin(), joint_collision.end());

    std_msgs::Float64MultiArray cart_collision_msg;
    cart_collision_msg.data.clear();
    cart_collision_msg.data.insert(cart_collision_msg.data.end(), cart_collision.begin(), cart_collision.end());

    // ROS_INFO_STREAM("cart_contact: " << cart_contact[0] << ", " << cart_contact[1] << ", " << cart_contact[2] << ", " << cart_contact[3] << ", " << cart_contact[4] << ", " << cart_contact[5]);
    // ROS_INFO_STREAM("cart_collision: " << cart_collision[0] << ", " << cart_collision[1] << ", " << cart_collision[2] << ", " << cart_collision[3] << ", " << cart_collision[4] << ", " << cart_collision[5]);

    std_msgs::Float64MultiArray torque_lower;
    torque_lower.data.resize(7);
    torque_lower.data = lower_torque_acc;
    std_msgs::Float64MultiArray torque_upper;
    torque_upper.data.resize(7);
    torque_upper.data = upper_torque_acc;
    std_msgs::Float64MultiArray force_lower;
    force_lower.data.resize(6);
    force_lower.data = lower_force_acc;
    std_msgs::Float64MultiArray force_upper;
    force_upper.data.resize(6);
    force_upper.data = upper_force_acc;

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

    Eigen::Vector3d current_position(ee_translation);
    Eigen::Quaterniond current_orientation(ee_linear);

    // -EE Pos: Set Desired and get current ee position and orientation--
    // Call line or plane (or nothing) object methods
    Eigen::Vector3d desired_position;
#ifdef HYBRID
    if (hybrid_mode_list[FREE_MODE_IDX])
    {
      desired_position = current_position;
    }
    else if (hybrid_mode_list[LINE_MODE_IDX] && line_plane_class_init)
    {
      desired_position = line->GetDesiredCrosstrackLocation(ee_translation, distance_param);
    }
    else if (hybrid_mode_list[PLANE_2D_MODE_IDX] && line_plane_class_init)
    {
      desired_position =
          plane_2d->GetDesiredCrosstrackLocation(ee_translation, distance_param_x, distance_param_y);
    }
    else if (hybrid_mode_list[CIRCLE_MODE_IDX] && circle_class_init)
    {
      desired_position = circle->GetDesiredCrosstrackLocation(ee_translation, theta_param);
    }
    else if (hybrid_mode_list[SPLINE_MODE_IDX] && spline_class_init)
    {
      spline->FindDesiredSplinePoint(ee_translation);
      desired_position = spline->GetBestPoint();
    }
    else if (hybrid_mode_list[PLANE_3D_MODE_IDX] && plane_3d_class_init)
    {
      plane_3d->FindDesired3dPlanePoint(ee_translation, T_plane_3d);
      desired_position = plane_3d->GetBestPoint();
    }
#endif
    Eigen::Vector3d desired_position_raw = desired_position;

#if !defined(HYBRID)
    // This if-else is for nothing case
    Eigen::Vector3d desired_traj(-0.0001 * (time_in_sec - 10), 0, 0);
    if (time_in_sec < 10)
    {
      Eigen::Vector3d x_offset(0.6, 0, 0);
      desired_position = line_plane_points[0] + x_offset;
    }
    else
    {
      // desired_position = ee_translation; // Free to move in space
      desired_position = desired_position + desired_traj; // Free to move in space
    }
#endif

    // ----------EE Pos: Compute position and orientation error-----------
    Eigen::Matrix<double, 6, 1> error;
    uint8_t pos_error_flag = 0;
    Eigen::Vector3d pos_err = desired_position - current_position;
    double pos_err_mag = pos_err.norm();
    double error_thresh = 0.03;
    double pos_err_scale = 1.0;
    if (pos_err_mag > error_thresh)
    {
      pos_err_scale = error_thresh / pos_err_mag;
      if (mode_switch_flag)
      {
        pos_error_flag = 1;
      }
    }
    pos_err *= pos_err_scale;
    if (!pos_error_flag)
      mode_switch_flag = 0;
    // If no geometry has been initialized, manually set error to zero to force the robot to remain still.
    if (line_plane_class_init && circle_class_init && spline_class_init && plane_3d_class_init)
    {
      error.head(3) << pos_err;
    }
    else
    {
      error.head(3).setZero();
    }

    // -----------EE Twist: Set desired twist in frenet frame-------------
    Eigen::Matrix<double, 6, 1> desired_velocity_frenet;
    desired_velocity_frenet << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0; // in order e_t,e_b,e_n,e_t,e_b,e_n

    // -------Compute instantaneous frenet frame and rotation matrix------
    // Call line or plane (or nothing) get unit direction methods
    Eigen::Vector3d e_t;
    Eigen::Vector3d e_t_plane;
#ifdef HYBRID
    if (hybrid_mode_list[FREE_MODE_IDX])
    {
      e_t << M_PI, 0.0, 0.0;
    }
    else if (hybrid_mode_list[LINE_MODE_IDX] && line_plane_class_init)
    {
      e_t = line->GetLineUnitDirection();
    }
    else if (hybrid_mode_list[PLANE_2D_MODE_IDX] && line_plane_class_init)
    {
      e_t = (plane_2d->GetPlaneUnitDirection()).col(0);
      e_t_plane = (plane_2d->GetPlaneUnitDirection()).col(2);
    }
    else if (hybrid_mode_list[CIRCLE_MODE_IDX] && circle_class_init)
    {
      e_t = circle->GetUnitDirection(theta_param);
    }
    else if (hybrid_mode_list[SPLINE_MODE_IDX] && spline_class_init)
    {
      e_t = spline->GetBestTangent();
    }
    else if (hybrid_mode_list[PLANE_3D_MODE_IDX] && plane_3d_class_init)
    {
      e_t = plane_3d->GetBestTangent();
    }
#endif
    Eigen::Vector3d e_fallback(0, 0, 1);
    Eigen::Vector3d e_n = -error.head(3);
    e_n -= e_n.dot(e_t) * e_t;
    if (e_n.norm() > 1e-6)
    {
      e_n.normalize();
    }
    else
    {
      e_n = e_fallback;
    }
    Eigen::Vector3d e_b = e_n.cross(e_t);
    e_b.normalize();
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

    Eigen::Vector3d x_unit = R3_3.col(0);
    Eigen::Vector3d y_unit = R3_3.col(1);
    Eigen::Vector3d z_unit = R3_3.col(2);

    Eigen::Matrix3d R_des = R3_3;
    if (rot_mode_list[UPRIGHT_IN_WS_IDX])
    {
      // rotate +180 about x
      Eigen::AngleAxisd rot(M_PI, Eigen::Vector3d::UnitX());
      R_des = rot.toRotationMatrix();
    }
    else if (rot_mode_list[TANGENT_TO_SHAPE_IDX])
    {
      if (z_unit.z() > 0)
      {
        // rotate +180 about x
        Eigen::AngleAxisd rot(M_PI, Eigen::Vector3d::UnitX());
        Eigen::Matrix3d R_rot = rot.toRotationMatrix();
        R_des = R3_3 * R_rot;
      }
    }

    Eigen::Isometry3d T_ = Eigen::Isometry3d::Identity();
    T_.translate(desired_position);
    T_.rotate(R_des);
    Eigen::Matrix4d T_des = T_.matrix();

    Eigen::Vector4d current_position_4d(current_position.x(), current_position.y(), current_position.z(), 1.0);
    Eigen::Vector4d current_position_4d_wall = T_des * current_position_4d;
    Eigen::Vector3d current_position_wall(current_position_4d_wall(0), current_position_4d_wall(1), current_position_4d_wall(2));

    Eigen::Matrix3d R_err = R_des * cur_R.transpose();
    Eigen::AngleAxisd aa(R_err);
    Eigen::Vector3d e_R = aa.angle() * aa.axis();
    Eigen::Vector3d e_R_ee = cur_R.transpose() * e_R;
    Eigen::Vector3d e_R_ee_raw = e_R_ee;

    Eigen::Quaterniond desired_orientation(R_des);

    if (desired_orientation.coeffs().dot(current_orientation.coeffs()) < 0.0)
    {
      current_orientation.coeffs() << -current_orientation.coeffs();
    }
    Eigen::Quaterniond error_quaternion(current_orientation.inverse() * desired_orientation);

    // Limit parameters
    double angle_limit = 0.075;

#ifdef HYBRID
    double max_angle = 0.0;
    if (hybrid_mode_list[LINE_MODE_IDX] || hybrid_mode_list[SPLINE_MODE_IDX])
    {
      // Find the max angle axis (ignoring x) for the desired rotation
      max_angle = std::max({abs(e_R_ee.y()), abs(e_R_ee.z())});
    }
    else if (hybrid_mode_list[PLANE_2D_MODE_IDX] || hybrid_mode_list[PLANE_3D_MODE_IDX])
    {
      // Find the max angle axis (ignoring z) for the desired rotation
      max_angle = std::max({abs(e_R_ee.x()), abs(e_R_ee.y())});
    }
    else
    {
      // Find the max angle axis for the desired rotation
      max_angle = std::max({abs(e_R_ee.x()), abs(e_R_ee.y()), abs(e_R_ee.z())});
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
    e_R_ee *= scale_factor;
    e_R = cur_R * e_R_ee;

    // Reconstruct limited rotation quaternion
    double e_theta = e_R.norm();
    Eigen::Quaterniond error_limited;
    if (e_theta < 1e-6)
    {
      error_limited.setIdentity();
    }
    else
    {
      Eigen::Vector3d e_axis = e_R / e_theta;
      error_limited = Eigen::AngleAxisd(e_theta, e_axis);
    }
    Eigen::Quaterniond q_desired = current_orientation * error_limited;
    // If no geometry has been initialized, manually set error to zero to force the robot to remain still.
    if (line_plane_class_init && circle_class_init && spline_class_init && plane_3d_class_init)
    {
      error.tail(3) << e_R;
    }
    else
    {
      error.tail(3).setZero();
    }
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
    K_switch(2, 2) = !hybrid_mode_list[FREE_MODE_IDX];
    if (control_mode_list[WALL_IDX])
    {
      if (hybrid_mode_list[PLANE_2D_MODE_IDX] || hybrid_mode_list[PLANE_3D_MODE_IDX])
      {
        K_switch(2, 2) = (current_position_wall.z() > -0.01);
      }
    }

    K_switch(3, 3) = !hybrid_mode_list[FREE_MODE_IDX];
    K_switch(4, 4) = !hybrid_mode_list[FREE_MODE_IDX];
    K_switch(5, 5) = !hybrid_mode_list[FREE_MODE_IDX];
    // Set switching matrix elements for orientation control
    if (rot_mode_list[TANGENT_TO_SHAPE_IDX])
    {
      // Turn off rotation about the ee x-axis in line or spline mode
      K_switch(3, 3) = !hybrid_mode_list[FREE_MODE_IDX] && ((!hybrid_mode_list[LINE_MODE_IDX] && !hybrid_mode_list[SPLINE_MODE_IDX]) || pos_error_flag);
      K_switch(4, 4) = !hybrid_mode_list[FREE_MODE_IDX];
      // Turn off rotation about the ee z-axis in plane mode
      K_switch(5, 5) = !hybrid_mode_list[FREE_MODE_IDX] && ((!hybrid_mode_list[PLANE_2D_MODE_IDX] && !hybrid_mode_list[PLANE_3D_MODE_IDX]) || pos_error_flag);
    }
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
    tau_d = jacobian.transpose() * (R * K_switch * K_des * R.transpose() * error +
                                    R * C_des * R.transpose() * (-jacobian * dq)) +
            coriolis;

    Eigen::Matrix<double, 6, 1> x_dot = -jacobian * dq;
    std_msgs::Float64MultiArray x_dot_msg;

    x_dot_msg.data.clear();

    x_dot_msg.data.insert(x_dot_msg.data.end(), x_dot.data(), x_dot.data() + x_dot.size());
    x_dot_pub_.publish(x_dot_msg);

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

    geometry_msgs::Vector3 rot_error_vec;
    rot_error_vec.x = abs(e_R_ee_raw[0]);
    rot_error_vec.y = abs(e_R_ee_raw[1]);
    rot_error_vec.z = abs(e_R_ee_raw[2]);
    rot_error_vec_pub_.publish(rot_error_vec);

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
    Eigen::Vector3d Fext;
    Fext << external_wrench[0], external_wrench[1], external_wrench[2];
    Eigen::Vector3d Fext_ee = R3_3 * Fext;
    // Eigen::Vector3d Fext_ee = Fext_ee_homogeneous.head<3>();
    // std::cout << "ee position: " << current_position_ee << std::endl;
    geometry_msgs::Vector3 Fext_ee_final;
    Fext_ee_final.x = Fext_ee.x();
    Fext_ee_final.y = Fext_ee.y();
    Fext_ee_final.z = Fext_ee.z();
    fext_ee_pub_.publish(Fext_ee_final);

    // Tauext in the EE frame
    Eigen::Vector4d Tauext;
    Tauext << external_wrench[3], external_wrench[4], external_wrench[5], 1.0;
    Eigen::Vector4d Tauext_ee = T * Tauext;
    geometry_msgs::Vector3 Tauext_ee_final;
    Tauext_ee_final.x = Tauext_ee.x();
    Tauext_ee_final.y = Tauext_ee.y();
    Tauext_ee_final.z = Tauext_ee.z();
    tauext_ee_pub_.publish(Tauext_ee_final);

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

    // Force lower and upper limits
    force_lower_pub_.publish(force_lower);
    force_upper_pub_.publish(force_upper);

    // Torque lower and upper limits
    torque_lower_pub_.publish(torque_lower);
    torque_upper_pub_.publish(torque_upper);

    // Saturate torque rate to avoid discontinuities
    tau_d << saturateTorqueRate(tau_d, tau_J_d);

    for (size_t i = 0; i < 7; ++i)
    {
      joint_handles_[i].setCommand(tau_d(i));
    }

    std_msgs::Float64MultiArray tau_d_msg;
    tau_d_msg.data.resize(tau_d.size());
    for (int i = 0; i < tau_d.size(); ++i)
    {
      tau_d_msg.data[i] = tau_d(i);
    }
    tau_d_pub_.publish(tau_d_msg);

    std_msgs::Float64MultiArray tau_hat_msg;
    tau_hat_msg.data.resize(external_torque.size());
    for (int i = 0; i < external_torque.size(); ++i)
    {
      tau_hat_msg.data[i] = external_torque[i];
    }
    tau_hat_pub_.publish(tau_hat_msg);

    joint_collision_pub_.publish(joint_collision_msg);
    cart_collision_pub_.publish(cart_collision_msg);
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

  void PandaCobotController::hybridModeCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg)
  {
    // Copy message data into global variable
    hybrid_mode_list = msg->data;
  }

  void PandaCobotController::orientationModeCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg)
  {
    // Copy message data into global variable
    rot_mode_list = msg->data;
  }

  void PandaCobotController::controlModeCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg)
  {
    // Copy message data into global variable
    control_mode_list = msg->data;
  }

  void PandaCobotController::linePlaneCallback(const cobot_experimental_controller::PointArray::ConstPtr &msg)
  {
    // We only want to init once
    if (!line_plane_init)
    {
      line_plane_points.reserve(msg->points.size());
      line_plane_init = true;
    }

    for (const auto &pt : msg->points)
    {
      // Convert geometry_msgs::Point -> Eigen::Vector3d
      Eigen::Vector3d vec(pt.x, pt.y, pt.z);
      line_plane_points.push_back(vec);
    }
  }

  void PandaCobotController::circleCallback(const cobot_experimental_controller::PointArray::ConstPtr &msg)
  {
    // We only want to init once
    if (!circle_init)
    {
      circle_points.reserve(msg->points.size());
      circle_init = true;
    }

    for (const auto &pt : msg->points)
    {
      // Convert geometry_msgs::Point -> Eigen::Vector3d
      Eigen::Vector3d vec(pt.x, pt.y, pt.z);
      circle_points.push_back(vec);
    }
  }

  void PandaCobotController::splineCallback(const cobot_experimental_controller::PointArray::ConstPtr &msg)
  {
    // We only want to init once
    if (!spline_init)
    {
      spline_points.reserve(msg->points.size());
      spline_init = true;
    }

    for (const auto &pt : msg->points)
    {
      // Convert geometry_msgs::Point -> Eigen::Vector3d
      Eigen::Vector3d vec(pt.x, pt.y, pt.z);
      spline_points.push_back(vec);
    }
  }

  void PandaCobotController::plane3dCallback(const cobot_experimental_controller::PointArray::ConstPtr &msg)
  {
    // We only want to init once
    if (!plane_3d_init)
    {
      plane_3d_points.reserve(msg->points.size());
      plane_3d_init = true;
    }

    for (const auto &pt : msg->points)
    {
      // Convert geometry_msgs::Point -> Eigen::Vector3d
      Eigen::Vector3d vec(pt.x, pt.y, pt.z);
      plane_3d_points.push_back(vec);
    }
  }

  void PandaCobotController::plane3dTCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
  {
    for (int i = 0; i < 16; ++i)
      T_plane_3d(i / 4, i % 4) = msg->data[i];
  }

  void PandaCobotController::modeSwitchFlagCallback(const std_msgs::UInt8::ConstPtr &msg)
  {
    if (msg->data)
    {
      mode_switch_flag = 1;
    }
  }

  std::vector<double> PandaCobotController::readCollisionThreshold(const ros::NodeHandle &nh, const std::string &name, const std::vector<double> &defaults)
  {
    std::vector<double> vals;
    std::string full_name = "collision_config/" + name;
    if (nh.getParam(full_name, vals))
    {
      if (vals.size() == defaults.size())
      {
        // ROS_INFO_STREAM("Read " << full_name << " with " << vals.size() << " entries.");
        return vals;
      }
      else
      {
        ROS_WARN_STREAM("Parameter " << full_name
                                     << " has size " << vals.size()
                                     << " but expected " << defaults.size()
                                     << ". Using defaults.");
      }
    }
    else
    {
      // ROS_INFO_STREAM("Parameter " << full_name << " not found. Using defaults.");
    }
    return defaults;
  }

} // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(cobot_experimental_controller::PandaCobotController,
                       controller_interface::ControllerBase)