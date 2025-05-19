#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
#include <cmath>
#include <limits>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

class Spline3d
{
public:
    Spline3d(Eigen::Vector3d p_1, Eigen::Vector3d p_2, Eigen::Vector3d p_3, Eigen::Vector3d p_4);
    Eigen::Vector3d GetDesiredCrosstrackLocation(Eigen::Vector3d current_position);
    Eigen::Vector3d GetSplineUnitDirection(Eigen::Vector3d current_positon);
    visualization_msgs::Marker GetSplineVis();

private:
    Eigen::Spline<double, 3> spline;
    int num_points;
    Eigen::Vector3d tangent;
    visualization_msgs::Marker spline_strip;

    void PrepToPub();
};