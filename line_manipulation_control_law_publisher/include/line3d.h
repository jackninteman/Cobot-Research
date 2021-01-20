#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <ecl/geometry.hpp>
#include <Eigen/Dense>
#include <cmath>

class Line3d
{
  public:
    // There are 2 ways to contruct a line presented here. One is from x,y,z param and 
    // another one is p_initial and p_final. Both are overloaded below 
    Line3d(std::vector<double> a, std::vector<double> b, std::vector<double> c) : 
        _x_param(a),
        _y_param(b),
        _z_param(c) {}
    Line3d(Eigen::Vector3d p_initial, Eigen::Vector3d p_final) :
        _p_initial(p_initial),
        _p_final(p_final) {}
    Eigen::Vector3d GetDesiredCrosstrackLocation(Eigen::Vector3d current_cartesian_position);
    Eigen::Vector3d GetDesiredCrosstrackLocation(Eigen::Vector3d current_cartesian_position, double &distance_param);
    Eigen::Vector3d GetDesiredPositionTrajectory(Eigen::Vector3d begin_cartesian_position, double time_in_sec, double time_final_in_sec);
    Eigen::Vector3d GetDesiredVelocityTrajectory(Eigen::Vector3d begin_cartesian_position, double time_in_sec, double time_final_in_sec);
    Eigen::Vector3d GetDesiredAccelerationTrajectory(Eigen::Vector3d begin_cartesian_position, double time_in_sec, double time_final_in_sec);
    Eigen::Vector3d GetLineUnitDirection() {return (_p_final - _p_initial)/(_p_final - _p_initial).norm();}
  
  private:
    std::vector<double> _x_param;
    std::vector<double> _y_param;
    std::vector<double> _z_param;
    Eigen::Vector3d _p_initial;
    Eigen::Vector3d _p_final;
    double GetOptimalLineParam(ecl::Array<double> X, Eigen::Vector3d current_cartesian_position);
    double GetDistance(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const; //Don't use this. It doesn't mean get distance between two points
};