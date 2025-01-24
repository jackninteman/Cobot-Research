#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <Eigen/Dense>
#include <cmath>

class Line3d
{
  public:
    Line3d(Eigen::Vector3d p_initial, Eigen::Vector3d p_final) :
        _p_initial(p_initial),
        _p_final(p_final) {}
    Eigen::Vector3d GetDesiredCrosstrackLocation(Eigen::Vector3d current_cartesian_position, double &distance_param);
    Eigen::Vector3d GetDesiredPositionTrajectory(Eigen::Vector3d begin_cartesian_position, double time_in_sec, double time_final_in_sec);
    Eigen::Vector3d GetDesiredVelocityTrajectory(Eigen::Vector3d begin_cartesian_position, double time_in_sec, double time_final_in_sec);
    Eigen::Vector3d GetDesiredAccelerationTrajectory(Eigen::Vector3d begin_cartesian_position, double time_in_sec, double time_final_in_sec);
    Eigen::Vector3d GetLineUnitDirection() {return (_p_final - _p_initial)/(_p_final - _p_initial).norm();}
  
  private:
    Eigen::Vector3d _p_initial;
    Eigen::Vector3d _p_final;
};