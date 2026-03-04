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

class Plane3d
{
public:
    Plane3d(std::vector<Eigen::Vector3d> sample_points);
    void FindDesired3dPlanePoint(Eigen::Vector3d current_position, Eigen::Matrix<double, 4, 4> T);
    visualization_msgs::Marker Get3dPlaneVis(Eigen::Matrix<double, 4, 4> T);
    Eigen::Vector3d GetBestPoint();
    Eigen::Vector3d GetBestTangent();

private:
    Eigen::Spline<double, 2> plane_3d;
    int num_points;
    Eigen::Vector3d tangent;
    visualization_msgs::Marker plane_3d_strip;
    Eigen::Vector3d bestPoint;
    Eigen::Vector3d bestTangent;

    void PrepToPub(Eigen::Matrix<double, 4, 4> T);
};