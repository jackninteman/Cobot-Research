#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>
// #include <ecl/geometry.hpp>
#include <Eigen/Dense>
#include <cmath>

class Circle3d
{
public:
  Circle3d(Eigen::Vector3d p_initial, Eigen::Vector3d p_final_approx, Eigen::Vector3d p_center);
  Eigen::Vector3d GetUnitDirection(const double &theta_param);
  Eigen::Vector3d GetDesiredCrosstrackLocation(Eigen::Vector3d current_cartesian_position, double &theta_param);
  double GetRadius();
  Eigen::MatrixXd GetRot();

private:
  Eigen::Vector3d _p_initial;
  Eigen::Vector3d _p_final_approx;
  Eigen::Vector3d _p_center;
  Eigen::Matrix<double, 3, 3> _R;
  double _radius;
};