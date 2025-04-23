#include "space_manipulation/circle3d.h"

Circle3d::Circle3d(Eigen::Vector3d p_initial, Eigen::Vector3d p_final_approx, Eigen::Vector3d p_center)
{
    _p_initial = p_initial;
    _p_final_approx = p_final_approx;
    _p_center = p_center;

    Eigen::Vector3d e_r = (_p_initial - _p_center) / (_p_initial - _p_center).norm();
    Eigen::Vector3d e_temp = (_p_final_approx - _p_center) / (_p_final_approx - _p_center).norm();
    Eigen::Vector3d e_z = e_r.cross(e_temp);
    Eigen::Vector3d e_theta = e_z.cross(e_r);
    _R << e_r, e_theta, e_z;

    _radius = (_p_initial - _p_center).norm();
}

Eigen::Vector3d Circle3d::GetUnitDirection(const double &theta_param)
{
    Eigen::Vector3d e_t(-std::sin(theta_param), std::cos(theta_param), 0.0);
    e_t = _R * e_t;
    return e_t;
}

Eigen::Vector3d Circle3d::GetDesiredCrosstrackLocation(Eigen::Vector3d current_cartesian_position, double &theta_param)
{
    Eigen::Vector3d current_cart_pos_circle_frame = _R.transpose() * (current_cartesian_position - _p_center);
    theta_param = std::atan2(current_cart_pos_circle_frame[1], current_cart_pos_circle_frame[0]);
    Eigen::Vector3d desired_position(_radius * std::cos(theta_param), _radius * std::sin(theta_param), 0.0);
    return _p_center + _R * desired_position;
}

double Circle3d::GetRadius()
{
    return _radius;
}

Eigen::MatrixXd Circle3d::GetRot()
{
    return _R;
}