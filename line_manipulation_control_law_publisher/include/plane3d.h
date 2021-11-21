#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <Eigen/Dense>
#include <cmath>

class Plane3d
{
  public:
    Plane3d(Eigen::Vector3d p_initial, Eigen::Vector3d p_2, Eigen::Vector3d p_3) :
        _p_initial(p_initial),
        _p_2(p_2),
        _p_3(p_3) {}
    Eigen::Vector3d GetDesiredCrosstrackLocation(Eigen::Vector3d current_cartesian_position, double &distance_param_x, double &distance_param_y);
    Eigen::Matrix3d GetPlaneUnitDirection();
  private:
    // We need 3 points to define a plane
    Eigen::Vector3d _p_initial;
    Eigen::Vector3d _p_2;
    Eigen::Vector3d _p_3;
};