#include "line3d.h"

Eigen::Vector3d Line3d::GetDesiredCrosstrackLocation(Eigen::Vector3d current_cartesian_position, double &distance_param)
{
    distance_param = ((_p_final - _p_initial).dot(current_cartesian_position - _p_initial))/(_p_final - _p_initial).norm();
    return _p_initial + distance_param*this->GetLineUnitDirection();
}

Eigen::Vector3d Line3d::GetDesiredPositionTrajectory(Eigen::Vector3d begin_cartesian_position, double time_in_sec, double time_final_in_sec)
{
    if (time_in_sec > time_final_in_sec) time_in_sec = time_final_in_sec;
    double D = (this->_p_final - begin_cartesian_position).norm();
    double s = 6.0*D/std::pow(time_final_in_sec,5)*std::pow(time_in_sec,5) + 
               -15.0*D/std::pow(time_final_in_sec,4)*std::pow(time_in_sec,4) +
               10.0*D/std::pow(time_final_in_sec,3)*std::pow(time_in_sec,3);
    
    return begin_cartesian_position + s*this->GetLineUnitDirection();
}
Eigen::Vector3d Line3d::GetDesiredVelocityTrajectory(Eigen::Vector3d begin_cartesian_position, double time_in_sec, double time_final_in_sec)
{
    if (time_in_sec > time_final_in_sec) time_in_sec = time_final_in_sec;
    double D = (this->_p_final - begin_cartesian_position).norm();
    double s_dot = 30.0*D/std::pow(time_final_in_sec,5)*std::pow(time_in_sec,4) + 
                   -60.0*D/std::pow(time_final_in_sec,4)*std::pow(time_in_sec,3) +
                   30.0*D/std::pow(time_final_in_sec,3)*std::pow(time_in_sec,2);
    
    return s_dot*this->GetLineUnitDirection();
}
Eigen::Vector3d Line3d::GetDesiredAccelerationTrajectory(Eigen::Vector3d begin_cartesian_position, double time_in_sec, double time_final_in_sec)
{
    if (time_in_sec > time_final_in_sec) time_in_sec = time_final_in_sec;
    double D = (this->_p_final - begin_cartesian_position).norm();
    double s_dot_dot = 120.0*D/std::pow(time_final_in_sec,5)*std::pow(time_in_sec,3) + 
                       -180.0*D/std::pow(time_final_in_sec,4)*std::pow(time_in_sec,2) +
                       60.0*D/std::pow(time_final_in_sec,3)*time_in_sec;
    
    return s_dot_dot*this->GetLineUnitDirection();
}