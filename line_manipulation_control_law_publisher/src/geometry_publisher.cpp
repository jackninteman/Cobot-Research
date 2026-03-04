//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------

#include <cstdint>
#include <cmath>
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <vector>
#include "cobot.h"
#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>
#include "line_manipulation_control_law_publisher/PointArray.h"

#include "space_manipulation/line3d.h"
#include "space_manipulation/circle3d.h"
#include "space_manipulation/plane2d.h"
#include "space_manipulation/spline3d.h"
#include "space_manipulation/plane3d.h"

//------------------------------------------------------------------------------
// VARIABLES
//------------------------------------------------------------------------------

ros::Publisher line_plane_pub;
ros::Publisher circle_pub;
ros::Publisher spline_pub;
ros::Publisher simple_3d_plane_pub;
ros::Publisher spline_vis_pub;
ros::Publisher simple_3d_plane_vis_pub;
ros::Publisher simple_3d_plane_T_pub;

// Points for line and plane
Eigen::Vector3d p_1(0.5, -0.25, 0.15);
Eigen::Vector3d p_2(0.4, 0.25, 0.3);
Eigen::Vector3d p_3(0.55, 0.0, 0.25);
std::vector<Eigen::Vector3d> line_plane_points = {
    p_1,
    p_2,
    p_3};
// Points for circle
Eigen::Vector3d p_c1(0.38, 0.25, 0.5);
Eigen::Vector3d p_c2(0.35, 0.0, 0.75);
Eigen::Vector3d p_cc(0.4, 0.0, 0.5);
std::vector<Eigen::Vector3d> circle_points = {
    p_c1,
    p_c2,
    p_cc};
// Points for spline
// You can define any number of points here
std::vector<Eigen::Vector3d> spline_points = {
    Eigen::Vector3d(0.6, -0.5, 0.1),
    Eigen::Vector3d(0.4, -0.25, 0.25),
    Eigen::Vector3d(0.6, 0.2, 0.35),
    Eigen::Vector3d(0.4, 0.5, 0.4),
    Eigen::Vector3d(0.6, 0.75, 0.5)};
// Points for simple 3d plane
// You can define any number of points here
// NOTE: Even though the points are 3D, always set x to 0.0 and handle rotation via T_plane
double plane_floor = 0.2;
double roll = 0;  // about X
double pitch = 0; //-M_PI / 12; // about Y
double yaw = 0;   // about Z
Eigen::Affine3d T_plane = Eigen::Affine3d::Identity();
Eigen::Vector3d t(0.5, 0.0, 0.0);
std::vector<Eigen::Vector3d>
    plane_3d_points = {
        Eigen::Vector3d(0.0, -0.75, 0.0 + plane_floor),
        Eigen::Vector3d(0.0, -0.6, 0.05 + plane_floor),
        Eigen::Vector3d(0.0, -0.5, 0.08 + plane_floor),
        Eigen::Vector3d(0.0, -0.2, 0.03 + plane_floor),
        Eigen::Vector3d(0.0, 0.1, 0.0 + plane_floor),
        Eigen::Vector3d(0.0, 0.3, 0.06 + plane_floor),
        Eigen::Vector3d(0.0, 0.6, 0.03 + plane_floor),
        Eigen::Vector3d(0.0, 0.75, 0.0 + plane_floor),
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "geometry");

    // Setup nh as unique pointer pointed to heap section
    // This node handle will be there as long as the main function keeps spinning
    std::unique_ptr<ros::NodeHandle> nh(new ros::NodeHandle);

    // Declare Publishers for all geometry points
    line_plane_pub = nh->advertise<line_manipulation_control_law_publisher::PointArray>("/line_plane_points", 10);
    circle_pub = nh->advertise<line_manipulation_control_law_publisher::PointArray>("/circle_points", 10);
    spline_pub = nh->advertise<line_manipulation_control_law_publisher::PointArray>("/spline_points", 10);
    simple_3d_plane_pub = nh->advertise<line_manipulation_control_law_publisher::PointArray>("/plane_3d_points", 10);
    spline_vis_pub = nh->advertise<visualization_msgs::Marker>("/spline", 10);
    simple_3d_plane_vis_pub = nh->advertise<visualization_msgs::Marker>("/plane_3d", 10);
    simple_3d_plane_T_pub = nh->advertise<std_msgs::Float64MultiArray>("plane_3d_T", 10);

    T_plane.translate(t);
    T_plane.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    T_plane.rotate(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
    T_plane.rotate(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));

    ros::Rate rate(30);

    while (ros::ok())
    {
        // Convert all geometry types to publishable message types and publish them

        line_manipulation_control_law_publisher::PointArray line_plane_points_;
        line_plane_points_.points.reserve(line_plane_points.size());
        for (const auto &ep : line_plane_points)
        {
            geometry_msgs::Point gp;
            gp.x = ep.x();
            gp.y = ep.y();
            gp.z = ep.z();
            line_plane_points_.points.push_back(gp);
        }
        line_plane_pub.publish(line_plane_points_);

        line_manipulation_control_law_publisher::PointArray circle_points_;
        circle_points_.points.reserve(circle_points.size());
        for (const auto &ep : circle_points)
        {
            geometry_msgs::Point gp;
            gp.x = ep.x();
            gp.y = ep.y();
            gp.z = ep.z();
            circle_points_.points.push_back(gp);
        }
        circle_pub.publish(circle_points_);

        line_manipulation_control_law_publisher::PointArray spline_points_;
        spline_points_.points.reserve(spline_points.size());
        for (const auto &ep : spline_points)
        {
            geometry_msgs::Point gp;
            gp.x = ep.x();
            gp.y = ep.y();
            gp.z = ep.z();
            spline_points_.points.push_back(gp);
        }
        spline_pub.publish(spline_points_);

        line_manipulation_control_law_publisher::PointArray simple_3d_plane_points_;
        simple_3d_plane_points_.points.reserve(plane_3d_points.size());
        for (const auto &ep : plane_3d_points)
        {
            geometry_msgs::Point gp;
            gp.x = ep.x();
            gp.y = ep.y();
            gp.z = ep.z();
            simple_3d_plane_points_.points.push_back(gp);
        }
        simple_3d_plane_pub.publish(simple_3d_plane_points_);

        std_msgs::Float64MultiArray msg;
        // Flatten the T matrix into the message
        for (int i = 0; i < T_plane.matrix().rows(); ++i)
        {
            for (int j = 0; j < T_plane.matrix().cols(); ++j)
            {
                msg.data.push_back(T_plane.matrix()(i, j)); // Add matrix element to the message
            }
        }
        simple_3d_plane_T_pub.publish(msg);

        // Publish the visualization marker for the spline
        Spline3d spline(spline_points);
        spline_vis_pub.publish(spline.GetSplineVis());

        // Publish the visualization marker for the simple 3d plane
        Plane3d plane_3d(plane_3d_points);
        simple_3d_plane_vis_pub.publish(plane_3d.Get3dPlaneVis(T_plane.matrix()));

        rate.sleep();
    }
}