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
#include "std_msgs/UInt8MultiArray.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>
#include <ros/callback_queue.h>
#include "space_manipulation/line3d.h"
#include "space_manipulation/circle3d.h"
#include "space_manipulation/plane2d.h"
#include "space_manipulation/spline3d.h"
#include "space_manipulation/plane3d.h"
#include "line_manipulation_control_law_publisher/PointArray.h"

//------------------------------------------------------------------------------
// DEFINES
//------------------------------------------------------------------------------

class HybridSubscriber
{
public:
  // Constructor
  HybridSubscriber(std::unique_ptr<ros::NodeHandle> &nh)
  {
    // Initialize the subscriber
    sub_ = nh->subscribe("/hybrid_mode", 1, &HybridSubscriber::callback, this);

    // Optionally initialize the vector with some default values
    hybrid_mode_data = DEFAULT_MODE;
  }

  // Callback function for handling the received message
  void callback(const std_msgs::UInt8MultiArray::ConstPtr &msg)
  {
    // Copy the data from the message into the received_vector_ member variable
    hybrid_mode_data = msg->data;
  }

  // Function to get the received vector
  const std::vector<uint8_t> &getReceivedVector() const
  {
    return hybrid_mode_data;
  }

private:
  ros::Subscriber sub_;                  // ROS subscriber
  std::vector<uint8_t> hybrid_mode_data; // Variable to store the received vector
};

class SplineSubcriber
{
public:
  SplineSubcriber(std::unique_ptr<ros::NodeHandle> &nh)
  {
    marker_sub_ = nh->subscribe("/spline", 1, &SplineSubcriber::markerCallback, this);
  }

  // Getter for full marker
  visualization_msgs::Marker getSplineMarker() const
  {
    return spline_marker_;
  }

private:
  ros::Subscriber marker_sub_;

  visualization_msgs::Marker spline_marker_;

  void markerCallback(const visualization_msgs::Marker::ConstPtr &msg)
  {
    spline_marker_ = *msg; // Deep copy of full marker
  }
};

class Plane3dSubscriber
{
public:
  Plane3dSubscriber(std::unique_ptr<ros::NodeHandle> &nh)
  {
    marker_sub_ = nh->subscribe("/plane_3d", 1, &Plane3dSubscriber::markerCallback, this);
  }

  // Getter for full marker
  visualization_msgs::Marker get3dPlaneMarker() const
  {
    return simple_3d_plane_marker_;
  }

private:
  ros::Subscriber marker_sub_;

  visualization_msgs::Marker simple_3d_plane_marker_;

  void markerCallback(const visualization_msgs::Marker::ConstPtr &msg)
  {
    simple_3d_plane_marker_ = *msg; // Deep copy of full marker
  }
};

Eigen::Vector3d des_pos;
// Points for line and 2D plane
std::vector<Eigen::Vector3d> line_plane_points;
bool line_plane_init = false;
// Points for circle
std::vector<Eigen::Vector3d> circle_points;
bool circle_init = false;
// Points for spline
std::vector<Eigen::Vector3d> spline_points;
bool spline_init = false;
// Points for 3D plane
std::vector<Eigen::Vector3d> plane_3d_points;
bool plane_3d_init = false;

geometry_msgs::Point eigenToPoint(const Eigen::Vector3d &vec);

void des_pos_callback(const geometry_msgs::Vector3::ConstPtr &msg);

void linePlaneCallback(const line_manipulation_control_law_publisher::PointArray::ConstPtr &msg);

void circleCallback(const line_manipulation_control_law_publisher::PointArray::ConstPtr &msg);

void splineCallback(const line_manipulation_control_law_publisher::PointArray::ConstPtr &msg);

void plane3dCallback(const line_manipulation_control_law_publisher::PointArray::ConstPtr &msg);

// ------ Need to clean up this code. This is just to show a straight line or circle on rviz------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "points_and_lines");

  // Setup n as unique pointer pointed to heap section
  // This node handle will be there as long as the main function keeps spinning
  std::unique_ptr<ros::NodeHandle> n(new ros::NodeHandle);

  ros::Publisher marker_pub = n->advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Subscriber des_pos_sub = n->subscribe("/des_pos_global", 10, des_pos_callback);

  // Setup subscriber to line_plane_points
  ros::Subscriber line_plane_sub = n->subscribe("/line_plane_points", 10, linePlaneCallback);

  // Setup subscriber to circle_points
  ros::Subscriber circle_sub = n->subscribe("/circle_points", 10, circleCallback);

  // Setup subscriber to spline_points
  ros::Subscriber spline_sub = n->subscribe("/spline_points", 10, splineCallback);

  // Setup subscriber to simple 3d plane points
  ros::Subscriber plane_3d_points_sub = n->subscribe("/plane_3d_points", 10, plane3dCallback);

  // Initialize the subscriber node to recieve hybrid mode status
  HybridSubscriber hybrid_subscriber(n);

  // Initialize the subscriber node to recieve spline info
  SplineSubcriber spline_subscriber(n);

  // Initialize the subscriber node to recieve simple 3d plane info
  Plane3dSubscriber plane_3d_subscriber(n);

  ros::Rate r(30);

  float f = 0.0;
  while (ros::ok())
  {
    // This prompts the node to always check the callback queue and update appropriately
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));

    if (line_plane_init && circle_init && spline_init && plane_3d_init)
    {
      visualization_msgs::Marker points,
          line_strip, line_list, plane, delete_marker, mode_text, circle, des_point;
      visualization_msgs::Marker spline = spline_subscriber.getSplineMarker();
      visualization_msgs::Marker plane_3d = plane_3d_subscriber.get3dPlaneMarker();
      points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = plane.header.frame_id = delete_marker.header.frame_id = mode_text.header.frame_id = circle.header.frame_id = des_point.header.frame_id = "panda_link0";
      points.header.stamp = line_strip.header.stamp = line_list.header.stamp = plane.header.stamp = delete_marker.header.stamp = mode_text.header.stamp = circle.header.stamp = des_point.header.stamp = ros::Time::now();
      points.ns = line_strip.ns = line_list.ns = plane.ns = delete_marker.ns = mode_text.ns = circle.ns = des_point.ns = "points_and_lines";
      points.action = line_strip.action = line_list.action = plane.action = mode_text.action = circle.action = des_point.action = visualization_msgs::Marker::ADD;
      delete_marker.action = visualization_msgs::Marker::DELETE;
      points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = des_point.pose.orientation.w = 1.0;

      points.id = 0;
      line_strip.id = 1;
      line_list.id = 2;
      plane.id = 3;
      mode_text.id = 4;
      circle.id = 5;
      spline.id = 6;
      plane_3d.id = 7;
      des_point.id = 8;

      points.type = visualization_msgs::Marker::POINTS;
      line_strip.type = visualization_msgs::Marker::LINE_STRIP;
      line_list.type = visualization_msgs::Marker::LINE_LIST;
      plane.type = visualization_msgs::Marker::CUBE;
      mode_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      circle.type = visualization_msgs::Marker::LINE_STRIP;
      des_point.type = visualization_msgs::Marker::POINTS;

      // MODE TEXT
      mode_text.pose.position.x = 0;
      mode_text.pose.position.y = -0.5;
      mode_text.pose.position.z = 0;

      mode_text.pose.orientation.w = 1.0;

      mode_text.scale.x = 0.1;
      mode_text.scale.y = 0.1;
      mode_text.scale.z = 0.1;

      mode_text.color.r = 1.0;
      mode_text.color.g = 0.0;
      mode_text.color.b = 0.0;
      mode_text.color.a = 1.0;

      // POINTS properties (green)
      points.scale.x = 0.05;
      points.scale.y = 0.05;

      points.color.g = 1.0f;
      points.color.a = 1.0;

      des_point.scale.x = 0.05;
      des_point.scale.y = 0.05;

      des_point.color.g = 1.0f;
      des_point.color.a = 1.0;

      // LINE properties (yellow)
      line_strip.scale.x = 0.01;

      line_strip.color.r = 1.0;
      line_strip.color.g = 1.0;
      line_strip.color.a = 1.0;

      // CIRCLE Properties (purple)
      int num_points = 100;

      circle.scale.x = 0.01;

      circle.color.r = 0.63;
      circle.color.g = 0.0;
      circle.color.b = 1.0;
      circle.color.a = 1.0;

      // PLANE properties (cyan)
      plane.scale.x = 2;
      plane.scale.y = 2;
      plane.scale.z = 0.001;

      plane.color.r = 0.0;
      plane.color.g = 1.0;
      plane.color.b = 1.0;
      plane.color.a = 0.8;

      // LINE_LIST properties (not used)
      line_list.scale.x = 0.1;

      line_list.color.r = 1.0;
      line_list.color.a = 1.0;

      Eigen::Vector3d &p_1 = line_plane_points[0];
      Eigen::Vector3d &p_2 = line_plane_points[1];
      Eigen::Vector3d &p_3 = line_plane_points[2];
      plane.pose.position.x = (p_1[0] + p_2[0]) / 2;
      plane.pose.position.y = (p_1[1] + p_2[1]) / 2;
      plane.pose.position.z = (p_1[2] + p_2[2]) / 2;
      Plane2d my_plane(p_1, p_2, p_3);
      Eigen::Quaterniond plane_orientation(my_plane.GetPlaneUnitDirection());
      plane.pose.orientation.w = plane_orientation.w();
      plane.pose.orientation.x = plane_orientation.x();
      plane.pose.orientation.y = plane_orientation.y();
      plane.pose.orientation.z = plane_orientation.z();

      Eigen::Vector3d p_difference((p_2 - p_1) / (p_2 - p_1).norm());

      Eigen::Vector3d &p_c1 = circle_points[0];
      Eigen::Vector3d &p_c2 = circle_points[1];
      Eigen::Vector3d &p_c = circle_points[2];
      Circle3d my_circle(p_c1, p_c2, p_c);
      double radius = my_circle.GetRadius();
      Eigen::Matrix<double, 3, 3> R_circle = my_circle.GetRot();
      // Create the vertices for the points and lines
      for (int i = -3; i <= 3; ++i)
      {
        float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
        float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

        geometry_msgs::Point p;
        p.x = p_1[0] + i * p_difference[0];
        p.y = p_1[1] + i * p_difference[1];
        p.z = p_1[2] + i * p_difference[2];

        // points.points.push_back(p);
        line_strip.points.push_back(p);

        // The line list needs two points for each line
        line_list.points.push_back(p);
        p.z += 1.0;
        line_list.points.push_back(p);
      }
      // Generate circle points and transform them
      for (int i = 0; i < num_points + 1; i++)
      {
        double theta = 2 * M_PI * i / num_points;
        Eigen::Vector3d local_point;
        local_point.x() = radius * cos(theta);
        local_point.y() = radius * sin(theta);
        local_point.z() = 0.0;

        // Rotate and translate the circle points
        Eigen::Vector3d global_point = R_circle * local_point + p_c;
        circle.points.push_back(eigenToPoint(global_point));
      }

#ifdef HYBRID
      const std::vector<uint8_t> &hybrid_mode_list = hybrid_subscriber.getReceivedVector();

      if (hybrid_mode_list.size() == NUM_GEO_MODES)
      {
        // geometry_msgs::Point des_p;
        // des_p.x = des_pos.x();
        // des_p.y = des_pos.y();
        // des_p.z = des_pos.z();
        // des_point.points.push_back(des_p);
        // marker_pub.publish(des_point);
        if (hybrid_mode_list[FREE_MODE_IDX])
        {
          // delete all pre-existing shapes/text
          delete_marker.id = 0;
          marker_pub.publish(delete_marker);
          delete_marker.id = 1;
          marker_pub.publish(delete_marker);
          delete_marker.id = 2;
          marker_pub.publish(delete_marker);
          delete_marker.id = 3;
          marker_pub.publish(delete_marker);
          delete_marker.id = 4;
          marker_pub.publish(delete_marker);
          delete_marker.id = 5;
          marker_pub.publish(delete_marker);
          delete_marker.id = 6;
          marker_pub.publish(delete_marker);
          delete_marker.id = 7;
          marker_pub.publish(delete_marker);
        }
        else if (hybrid_mode_list[LINE_MODE_IDX])
        {
          // delete all pre-existing shapes/text
          delete_marker.id = 0;
          marker_pub.publish(delete_marker);
          delete_marker.id = 3;
          marker_pub.publish(delete_marker);
          delete_marker.id = 4;
          marker_pub.publish(delete_marker);
          delete_marker.id = 5;
          marker_pub.publish(delete_marker);
          delete_marker.id = 6;
          marker_pub.publish(delete_marker);
          delete_marker.id = 7;
          marker_pub.publish(delete_marker);

          // Set points
          geometry_msgs::Point p1;
          p1.x = p_1.x();
          p1.y = p_1.y();
          p1.z = p_1.z();
          geometry_msgs::Point p2;
          p2.x = p_2.x();
          p2.y = p_2.y();
          p2.z = p_2.z();
          points.points.push_back(p1);
          points.points.push_back(p2);

          mode_text.text = "LINE MODE";
          marker_pub.publish(mode_text);
          marker_pub.publish(line_strip);
          // Optionally display the user defined points
          // marker_pub.publish(points);
        }
        else if (hybrid_mode_list[PLANE_2D_MODE_IDX])
        {
          // delete all pre-existing shapes/text
          delete_marker.id = 0;
          marker_pub.publish(delete_marker);
          delete_marker.id = 1;
          marker_pub.publish(delete_marker);
          delete_marker.id = 4;
          marker_pub.publish(delete_marker);
          delete_marker.id = 5;
          marker_pub.publish(delete_marker);
          delete_marker.id = 6;
          marker_pub.publish(delete_marker);
          delete_marker.id = 7;
          marker_pub.publish(delete_marker);

          // Set points
          geometry_msgs::Point p1;
          p1.x = p_1.x();
          p1.y = p_1.y();
          p1.z = p_1.z();
          geometry_msgs::Point p2;
          p2.x = p_2.x();
          p2.y = p_2.y();
          p2.z = p_2.z();
          geometry_msgs::Point p3;
          p3.x = p_3.x();
          p3.y = p_3.y();
          p3.z = p_3.z();
          points.points.push_back(p1);
          points.points.push_back(p2);
          points.points.push_back(p3);

          mode_text.text = "PLANE MODE";
          marker_pub.publish(mode_text);
          marker_pub.publish(plane);
          // Optionally display the user defined points
          // marker_pub.publish(points);
        }
        else if (hybrid_mode_list[CIRCLE_MODE_IDX])
        {
          // delete all pre-existing shapes/text
          delete_marker.id = 0;
          marker_pub.publish(delete_marker);
          delete_marker.id = 1;
          marker_pub.publish(delete_marker);
          delete_marker.id = 3;
          marker_pub.publish(delete_marker);
          delete_marker.id = 4;
          marker_pub.publish(delete_marker);
          delete_marker.id = 6;
          marker_pub.publish(delete_marker);
          delete_marker.id = 7;
          marker_pub.publish(delete_marker);

          mode_text.text = "CIRCLE MODE";
          marker_pub.publish(mode_text);
          marker_pub.publish(circle);
        }
        else if (hybrid_mode_list[SPLINE_MODE_IDX])
        {
          // delete all pre-existing shapes/text
          delete_marker.id = 0;
          marker_pub.publish(delete_marker);
          delete_marker.id = 1;
          marker_pub.publish(delete_marker);
          delete_marker.id = 3;
          marker_pub.publish(delete_marker);
          delete_marker.id = 4;
          marker_pub.publish(delete_marker);
          delete_marker.id = 5;
          marker_pub.publish(delete_marker);
          delete_marker.id = 7;
          marker_pub.publish(delete_marker);

          // Set points
          for (int i = 0; i < spline_points.size(); i++)
          {
            geometry_msgs::Point p;
            p.x = spline_points[i].x();
            p.y = spline_points[i].y();
            p.z = spline_points[i].z();
            points.points.push_back(p);
          }

          mode_text.text = "SPLINE MODE";
          marker_pub.publish(mode_text);
          marker_pub.publish(spline);
          // Optionally display the user defined points
          // marker_pub.publish(points);
        }
        else if (hybrid_mode_list[PLANE_3D_MODE_IDX])
        {
          // delete all pre-existing shapes/text
          delete_marker.id = 0;
          marker_pub.publish(delete_marker);
          delete_marker.id = 1;
          marker_pub.publish(delete_marker);
          delete_marker.id = 3;
          marker_pub.publish(delete_marker);
          delete_marker.id = 4;
          marker_pub.publish(delete_marker);
          delete_marker.id = 5;
          marker_pub.publish(delete_marker);
          delete_marker.id = 6;
          marker_pub.publish(delete_marker);

          // Set points
          for (int i = 0; i < plane_3d_points.size(); i++)
          {
            geometry_msgs::Point p;
            p.x = plane_3d_points[i].x();
            p.y = plane_3d_points[i].y();
            p.z = plane_3d_points[i].z();
            points.points.push_back(p);
          }

          mode_text.text = "3D PLANE MODE";
          marker_pub.publish(mode_text);
          marker_pub.publish(plane_3d);
          // Optionally display the user defined points
          // marker_pub.publish(points);
        }
      }
      else
      {
        ROS_INFO("ERROR: hybrid mode list is incorrect size. Expected size %d, got size %d", NUM_GEO_MODES, static_cast<int>(hybrid_mode_list.size()));
      }

#endif
    }

    r.sleep();

    f += 0.04;
  }
}

// Helper function for forming the circle
geometry_msgs::Point eigenToPoint(const Eigen::Vector3d &vec)
{
  geometry_msgs::Point p;
  p.x = vec.x();
  p.y = vec.y();
  p.z = vec.z();
  return p;
}

void des_pos_callback(const geometry_msgs::Vector3::ConstPtr &msg)
{
  des_pos << msg->x, msg->y, msg->z;
}

void linePlaneCallback(const line_manipulation_control_law_publisher::PointArray::ConstPtr &msg)
{
  // We only want to init once
  if (!line_plane_init)
  {
    line_plane_points.reserve(msg->points.size());
    line_plane_init = true;
  }

  for (const auto &pt : msg->points)
  {
    // Convert geometry_msgs::Point -> Eigen::Vector3d
    Eigen::Vector3d vec(pt.x, pt.y, pt.z);
    line_plane_points.push_back(vec);
  }
}

void circleCallback(const line_manipulation_control_law_publisher::PointArray::ConstPtr &msg)
{
  // We only want to init once
  if (!circle_init)
  {
    circle_points.reserve(msg->points.size());
    circle_init = true;
  }

  for (const auto &pt : msg->points)
  {
    // Convert geometry_msgs::Point -> Eigen::Vector3d
    Eigen::Vector3d vec(pt.x, pt.y, pt.z);
    circle_points.push_back(vec);
  }
}

void splineCallback(const line_manipulation_control_law_publisher::PointArray::ConstPtr &msg)
{
  // We only want to init once
  if (!spline_init)
  {
    spline_points.reserve(msg->points.size());
    spline_init = true;
  }

  for (const auto &pt : msg->points)
  {
    // Convert geometry_msgs::Point -> Eigen::Vector3d
    Eigen::Vector3d vec(pt.x, pt.y, pt.z);
    spline_points.push_back(vec);
  }
}

void plane3dCallback(const line_manipulation_control_law_publisher::PointArray::ConstPtr &msg)
{
  // We only want to init once
  if (!plane_3d_init)
  {
    plane_3d_points.reserve(msg->points.size());
    plane_3d_init = true;
  }

  for (const auto &pt : msg->points)
  {
    // Convert geometry_msgs::Point -> Eigen::Vector3d
    Eigen::Vector3d vec(pt.x, pt.y, pt.z);
    plane_3d_points.push_back(vec);
  }
}