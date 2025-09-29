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

namespace cobot_experimental_controller
{

#define HYBRID

#define NUM_MODES (4)

#define LINE_MODE_IDX (0)
#define PLANE_MODE_IDX (1)
#define CIRCLE_MODE_IDX (2)
#define SPLINE_MODE_IDX (3)

#define DEFAULT_MODE std::vector<uint8_t>({1, 0, 0, 0})

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

    Eigen::Vector3d ext_force_joystick;
    bool start_flag = true;
    double time_begin_in_sec;
    Eigen::Vector3d begin_cartesian_position;

    std::vector<uint8_t> hybrid_mode_list = DEFAULT_MODE;
    int desired_mode_idx = 0;

    // Equilibrium pose subscriber
    ros::Subscriber sub_equilibrium_pose_;
    ros::Publisher pose_pub_;
    ros::Publisher traj_pub_;
    ros::Publisher fext_ee_pub_;
    ros::Publisher cur_pos_ee_pub_;
    ros::Publisher des_pos_ee_pub_;
    ros::Publisher k_switch_pub_;
    ros::Publisher hybrid_mode_pub_;
    ros::Publisher spline_pub;
    ros::Subscriber joystickSubscriber;
    void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void JoystickFeedback(const sensor_msgs::Joy::ConstPtr &joystickhandlePtr_);
    uint8_t CheckOctant(Eigen::Vector3d e_t);
  };

} // namespace franka_example_controllers