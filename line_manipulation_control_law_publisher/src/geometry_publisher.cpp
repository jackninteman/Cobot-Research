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
#include "space_manipulation/plane3d.h"
#include "space_manipulation/spline3d.h"

//------------------------------------------------------------------------------
// VARIABLES
//------------------------------------------------------------------------------

ros::Publisher line_plane_pub;
ros::Publisher circle_pub;
ros::Publisher spline_pub;
ros::Publisher spline_vis_pub;

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
    Eigen::Vector3d(0.5, -0.25, 0.25),
    Eigen::Vector3d(0.6, 0.2, 0.3),
    Eigen::Vector3d(0.5, 0.5, 0.5),
    Eigen::Vector3d(0.6, 0.75, 0.6)};

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
    spline_vis_pub = nh->advertise<visualization_msgs::Marker>("/spline", 10);

    // Set rosparameters for line visualization
    nh->setParam("/p_initial_x", p_1[0]);
    nh->setParam("/p_initial_y", p_1[1]);
    nh->setParam("/p_initial_z", p_1[2]);
    nh->setParam("/p_final_x", p_2[0]);
    nh->setParam("/p_final_y", p_2[1]);
    nh->setParam("/p_final_z", p_2[2]);
    nh->setParam("/p_plane_x", p_3[0]);
    nh->setParam("/p_plane_y", p_3[1]);
    nh->setParam("/p_plane_z", p_3[2]);

    // Set rosparameters for plane visualization
    Plane3d plane(p_1, p_2, p_3);
    Eigen::Quaterniond plane_orientation(plane.GetPlaneUnitDirection());
    nh->setParam("/plane_orientation_w", plane_orientation.w());
    nh->setParam("/plane_orientation_x", plane_orientation.x());
    nh->setParam("/plane_orientation_y", plane_orientation.y());
    nh->setParam("/plane_orientation_z", plane_orientation.z());

    // Set rosparameters for circle visualization
    nh->setParam("/p_center_x", p_cc[0]);
    nh->setParam("/p_center_y", p_cc[1]);
    nh->setParam("/p_center_z", p_cc[2]);
    // Must flatten the R matrix so it can be set as a rosparameter
    Circle3d circle(p_c1, p_c2, p_cc);
    Eigen::MatrixXd R_circle = circle.GetRot();
    std::vector<double> R_flat = {R_circle(0, 0), R_circle(0, 1), R_circle(0, 2),
                                  R_circle(1, 0), R_circle(1, 1), R_circle(1, 2),
                                  R_circle(2, 0), R_circle(2, 1), R_circle(2, 2)};
    nh->setParam("/circle_rad", circle.GetRadius());
    nh->setParam("/circle_rot", R_flat);

    // Set rosparameters for spline visualization
    // Must flatten the spline points matrix so it can be set as a rosparameter
    std::vector<double> spline_points_flat;
    for (int i = 0; i < spline_points.size(); i++)
    {
        for (int j = 0; j < 3; j++)
        {
            spline_points_flat.push_back(spline_points[i][j]);
        }
    }
    nh->setParam("/spline_points", spline_points_flat);

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

        // Publish the visualization marker for the spline
        Spline3d spline(spline_points);
        spline_vis_pub.publish(spline.GetSplineVis());

        rate.sleep();
    }
}