#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <ecl/geometry.hpp>
#include <Eigen/Dense>
#include <cmath>

class Line3d
{
  public:
    Line3d(std::vector<double> a, std::vector<double> b, std::vector<double> c) : 
        _x_param(a),
        _y_param(b),
        _z_param(c) {}
    Eigen::Vector3d GetDesiredCrosstrackLocation(Eigen::Vector3d current_cartesian_position);
  private:
    std::vector<double> _x_param;
    std::vector<double> _y_param;
    std::vector<double> _z_param;
    double GetOptimalLineParam(ecl::Array<double> X, Eigen::Vector3d current_cartesian_position);
    double GetDistance(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const;
};