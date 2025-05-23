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
    void FindDesiredSplinePoint(Eigen::Vector3d current_position);
    visualization_msgs::Marker GetSplineVis();
    Eigen::Vector3d GetBestPoint();
    Eigen::Vector3d GetBestTangent();

private:
    Eigen::Spline<double, 3> spline;
    int num_points;
    Eigen::Vector3d tangent;
    visualization_msgs::Marker spline_strip;
    Eigen::Vector3d bestSplinePoint;
    Eigen::Vector3d bestTangent;

    void PrepToPub();
};