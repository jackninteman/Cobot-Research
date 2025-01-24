#include "space_manipulation/plane3d.h"

Eigen::Vector3d Plane3d::GetDesiredCrosstrackLocation(Eigen::Vector3d current_cartesian_position, double &distance_param_x, double &distance_param_y)
{
    Eigen::Matrix3d R3(GetPlaneUnitDirection());
    Eigen::Matrix3d R3_T(R3.transpose());
    Eigen::Matrix3d I_1_1_0;
    I_1_1_0 << 1,0,0,
               0,1,0,
               0,0,0;
    Eigen::Vector3d vec_x(1,0,0);
    Eigen::Vector3d vec_y(0,1,0);
    distance_param_x = (R3_T*(current_cartesian_position - _p_initial)).dot(vec_x);
    distance_param_x = (R3_T*(current_cartesian_position - _p_initial)).dot(vec_y);
    return _p_initial + R3*I_1_1_0*R3_T*(current_cartesian_position - _p_initial);
}

Eigen::Matrix3d Plane3d::GetPlaneUnitDirection() {
    Eigen::Vector3d e_x((_p_2 - _p_initial)/(_p_2 - _p_initial).norm());
    Eigen::Vector3d e_in_plane((_p_3 - _p_initial));
    Eigen::Vector3d e_z(( e_x.cross(e_in_plane) )/( e_x.cross(e_in_plane) ).norm());
    Eigen::Vector3d e_y(e_z.cross(e_x));
    Eigen::Matrix3d R3;
    R3 << e_x, e_y, e_z;
    return R3;
}