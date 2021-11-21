#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>

#include <cmath>

// ------ Need to clean up this code. This is just to show a straight line or circle on rviz------
int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate r(30);

  float f = 0.0;
  while (ros::ok())
  {

    visualization_msgs::Marker points, line_strip, line_list, plane;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = plane.header.frame_id = "/world";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = plane.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = plane.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = plane.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;



    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;
    plane.id = 3;



    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    plane.type = visualization_msgs::Marker::CUBE;

    



    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.01;
    line_list.scale.x = 0.1;

    // PLANE
    plane.scale.x = 5;
    plane.scale.y = 5;
    plane.scale.z = 0;



    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is yellow
    line_strip.color.r = 1.0;
    line_strip.color.g = 1.0;
    line_strip.color.a = 1.0;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    // Plane is cyan
    plane.color.g = 1.0;
    plane.color.b = 1.0;
    plane.color.a = 0.8;


    Eigen::Vector3d p_initial;
    Eigen::Vector3d p_final;
    Eigen::Quaterniond plane_orientation;
    n.getParam("p_initial_x", p_initial[0]);
    n.getParam("p_initial_y", p_initial[1]);
    n.getParam("p_initial_z", p_initial[2]);
    n.getParam("p_final_x", p_final[0]);
    n.getParam("p_final_y", p_final[1]);
    n.getParam("p_final_z", p_final[2]);
    n.getParam("plane_orientation_w", plane_orientation.w());
    n.getParam("plane_orientation_x", plane_orientation.x());
    n.getParam("plane_orientation_y", plane_orientation.y());
    n.getParam("plane_orientation_z", plane_orientation.z());
    plane.pose.position.x = p_initial[0];
    plane.pose.position.y = p_initial[1];
    plane.pose.position.z = p_initial[2];
    plane.pose.orientation.w = plane_orientation.w();
    plane.pose.orientation.x = plane_orientation.x();
    plane.pose.orientation.y = plane_orientation.y();
    plane.pose.orientation.z = plane_orientation.z();
    Eigen::Vector3d p_difference((p_final-p_initial)/(p_final-p_initial).norm());
    int sample=20;
    // Create the vertices for the points and lines
    for (int i = -3; i <= 3; ++i)
    {
      float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
      float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

      

      geometry_msgs::Point p;
      p.x = p_initial[0] + i*p_difference[0];
      p.y = p_initial[1] + i*p_difference[1];
      p.z = p_initial[2] + i*p_difference[2];

      points.points.push_back(p);
      line_strip.points.push_back(p);

      // The line list needs two points for each line
      line_list.points.push_back(p);
      p.z += 1.0;
      line_list.points.push_back(p);
    }


    //marker_pub.publish(points);
    //marker_pub.publish(line_strip);
    //marker_pub.publish(line_list);
    marker_pub.publish(plane);

    r.sleep();

    f += 0.04;
  }
}