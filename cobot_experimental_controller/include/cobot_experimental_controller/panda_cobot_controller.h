// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <sensor_msgs/Joy.h>
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt8.h"
#include "cobot_experimental_controller/PointArray.h"

namespace cobot_experimental_controller
{

#define HYBRID

#define NUM_GEO_MODES (6)

#define FREE_MODE_IDX (0)
#define LINE_MODE_IDX (1)
#define PLANE_2D_MODE_IDX (2)
#define CIRCLE_MODE_IDX (3)
#define SPLINE_MODE_IDX (4)
#define PLANE_3D_MODE_IDX (5)

#define DEFAULT_MODE std::vector<uint8_t>({1, 0, 0, 0, 0, 0})

#define NUM_ROT_MODES (2)

#define UPRIGHT_IN_WS_IDX (0)
#define TANGENT_TO_SHAPE_IDX (1)

#define DEFAULT_ROT_MODE std::vector<uint8_t>({1, 0})

#define NUM_CONTROL_MODES (2)

#define CROSSTRACK_IDX (0)
#define WALL_IDX (1)

#define DEFAULT_CONTROL_MODE std::vector<uint8_t>({1, 0})

  class PandaCobotController : public controller_interface::MultiInterfaceController<
                                   franka_hw::FrankaModelInterface,
                                   hardware_interface::EffortJointInterface,
                                   franka_hw::FrankaStateInterface>
  {
  public:
    bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
    void starting(const ros::Time &) override;
    void update(const ros::Time &, const ros::Duration &period) override;

  private:
    // Saturation
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
        const Eigen::Matrix<double, 7, 1> &tau_J_d); // NOLINT (readability-identifier-naming)

    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;

    double filter_params_{0.005};
    double nullspace_stiffness_{20.0};
    double nullspace_stiffness_target_{20.0};
    const double delta_tau_max_{1.0};
    ros::Time start_time_;
    bool start_flag_{true};
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
    Eigen::Matrix<double, 6, 6> stiffness;
    Eigen::Matrix<double, 6, 6> stiffness_switch;
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_;
    Eigen::Matrix<double, 6, 6> damping;
    Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
    Eigen::Matrix<double, 7, 1> q_d_nullspace_;
    Eigen::Vector3d position_d_;
    Eigen::Quaterniond orientation_d_;
    Eigen::Vector3d position_d_target_;
    Eigen::Quaterniond orientation_d_target_;

    bool start_flag = true;
    double time_begin_in_sec;
    Eigen::Vector3d begin_cartesian_position;

    std::vector<uint8_t> hybrid_mode_list = DEFAULT_MODE;
    int desired_mode_idx = LINE_MODE_IDX;

    std::vector<uint8_t> rot_mode_list = DEFAULT_ROT_MODE;
    std::vector<uint8_t> control_mode_list = DEFAULT_CONTROL_MODE;
    uint8_t mode_switch_flag = 1;

    std::vector<double> lower_torque_acc;
    std::vector<double> upper_torque_acc;
    std::vector<double> lower_torque_nom;
    std::vector<double> upper_torque_nom;
    std::vector<double> lower_force_acc;
    std::vector<double> upper_force_acc;
    std::vector<double> lower_force_nom;
    std::vector<double> upper_force_nom;

    std::string robot_address = "192.168.1.200";

    // Equilibrium pose subscriber
    ros::Subscriber sub_equilibrium_pose_;
    ros::Subscriber hybrid_mode_sub;
    ros::Subscriber orientation_mode_sub;
    ros::Subscriber control_mode_sub;
    ros::Subscriber line_plane_sub;
    ros::Subscriber circle_sub;
    ros::Subscriber spline_sub;
    ros::Subscriber plane_3d_points_sub;
    ros::Subscriber plane_3d_T_sub;
    ros::Subscriber mode_switch_flag_sub;
    ros::Publisher pose_pub_;
    ros::Publisher traj_pub_;
    ros::Publisher fext_ee_pub_;
    ros::Publisher tauext_ee_pub_;
    ros::Publisher cur_pos_ee_pub_;
    ros::Publisher des_pos_ee_pub_;
    ros::Publisher rot_error_vec_pub_;
    ros::Publisher k_switch_pub_;
    ros::Publisher torque_lower_pub_;
    ros::Publisher torque_upper_pub_;
    ros::Publisher force_lower_pub_;
    ros::Publisher force_upper_pub_;
    ros::Publisher tau_d_pub_;
    ros::Publisher tau_hat_pub_;
    ros::Publisher time_now_pub_;
    ros::Publisher joint_collision_pub_;
    ros::Publisher cart_collision_pub_;
    ros::Publisher x_dot_pub_;
    void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void hybridModeCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg);
    void orientationModeCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg);
    void controlModeCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg);
    void linePlaneCallback(const cobot_experimental_controller::PointArray::ConstPtr &msg);
    void circleCallback(const cobot_experimental_controller::PointArray::ConstPtr &msg);
    void splineCallback(const cobot_experimental_controller::PointArray::ConstPtr &msg);
    void plane3dCallback(const cobot_experimental_controller::PointArray::ConstPtr &msg);
    void plane3dTCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void modeSwitchFlagCallback(const std_msgs::UInt8::ConstPtr &msg);
    uint8_t CheckOctant(Eigen::Vector3d e_t);
    std::vector<double> readCollisionThreshold(const ros::NodeHandle &nh, const std::string &name, const std::vector<double> &defaults);
  };

} // namespace franka_example_controllers