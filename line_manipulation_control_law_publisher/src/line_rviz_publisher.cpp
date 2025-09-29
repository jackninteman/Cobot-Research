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

//------------------------------------------------------------------------------
// DEFINES
//------------------------------------------------------------------------------

class HybridSubscriber
{
public:
  // Constructor
  HybridSubscriber(ros::NodeHandle &nh)
  {
    // Initialize the subscriber
    sub_ = nh.subscribe("/hybrid_mode", 1, &HybridSubscriber::callback, this);

    // Optionally initialize the vector with some default values
    hybrid_mode_data = DEFAULT_MODE;
  }

  // Callback function for handling the received message
  void callback(const std_msgs::UInt8MultiArray::ConstPtr &msg)
  {
    // Copy the data from the message into the received_vector_ member variable
    hybrid_mode_data = msg->data;

    // // Print the vector to the console (for debugging)
    // ROS_INFO("Received vector: ");
    // for (size_t i = 0; i < hybrid_mode_data.size(); ++i)
    // {
    //   ROS_INFO("received_vector_[%zu] = %d", i, hybrid_mode_data[i]);
    // }
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
  SplineSubcriber(ros::NodeHandle &nh)
  {
    marker_sub_ = nh.subscribe("/spline", 1, &SplineSubcriber::markerCallback, this);
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

    //   ROS_INFO("Received marker with %lu points, ID = %d",
    //            msg->points.size(), msg->id);
  }
};

// Helper function for forming the circle
geometry_msgs::Point eigenToPoint(const Eigen::Vector3d &vec)
{
  geometry_msgs::Point p;
  p.x = vec.x();
  p.y = vec.y();
  p.z = vec.z();
  return p;
}

// ------ Need to clean up this code. This is just to show a straight line or circle on rviz------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  // Initialize the subscriber node to recieve hybrid mode status
  HybridSubscriber hybrid_subscriber(n);

  // Initialize the subscriber node to recieve spline info
  SplineSubcriber spline_subscriber(n);

  ros::Rate r(30);

  float f = 0.0;
  while (ros::ok())
  {
    // This prompts the node to always check the callback queue and update appropriately
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));

    visualization_msgs::Marker points,
        line_strip, line_list, plane, delete_marker, mode_text, circle;
    visualization_msgs::Marker spline = spline_subscriber.getSplineMarker();
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = plane.header.frame_id = delete_marker.header.frame_id = mode_text.header.frame_id = circle.header.frame_id = "panda_link0";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = plane.header.stamp = delete_marker.header.stamp = mode_text.header.stamp = circle.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = plane.ns = delete_marker.ns = mode_text.ns = circle.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = plane.action = mode_text.action = circle.action = visualization_msgs::Marker::ADD;
    delete_marker.action = visualization_msgs::Marker::DELETE;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;
    plane.id = 3;
    mode_text.id = 4;
    circle.id = 5;
    spline.id = 6;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    plane.type = visualization_msgs::Marker::CUBE;
    mode_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    circle.type = visualization_msgs::Marker::LINE_STRIP;

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

    // LINE properties (yellow)
    line_strip.scale.x = 0.01;

    line_strip.color.r = 1.0;
    line_strip.color.g = 1.0;
    line_strip.color.a = 1.0;

    // CIRCLE Properties (purple)
    double radius;
    std::vector<double> R_flat;
    Eigen::Matrix<double, 3, 3> R_circle;
    Eigen::Vector3d p_c;
    int num_points = 100;

    circle.scale.x = 0.01;

    circle.color.r = 0.63;
    circle.color.g = 0.0;
    circle.color.b = 1.0;
    circle.color.a = 1.0;

    // PLANE properties (cyan)
    plane.scale.x = 5;
    plane.scale.y = 5;
    plane.scale.z = 0.001;

    plane.color.r = 0.0;
    plane.color.g = 1.0;
    plane.color.b = 1.0;
    plane.color.a = 0.8;

    // LINE_LIST properties (not used)
    line_list.scale.x = 0.1;

    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    Eigen::Vector3d p_1;
    Eigen::Vector3d p_2;
    Eigen::Vector3d p_3;
    Eigen::Quaterniond plane_orientation;
    std::vector<double> spline_points;
    n.getParam("p_initial_x", p_1[0]);
    n.getParam("p_initial_y", p_1[1]);
    n.getParam("p_initial_z", p_1[2]);
    n.getParam("p_final_x", p_2[0]);
    n.getParam("p_final_y", p_2[1]);
    n.getParam("p_final_z", p_2[2]);
    n.getParam("p_plane_x", p_3[0]);
    n.getParam("p_plane_y", p_3[1]);
    n.getParam("p_plane_z", p_3[2]);
    n.getParam("plane_orientation_w", plane_orientation.w());
    n.getParam("plane_orientation_x", plane_orientation.x());
    n.getParam("plane_orientation_y", plane_orientation.y());
    n.getParam("plane_orientation_z", plane_orientation.z());
    n.getParam("spline_points", spline_points);
    plane.pose.position.x = p_1[0];
    plane.pose.position.y = p_1[1];
    plane.pose.position.z = p_1[2];
    plane.pose.orientation.w = plane_orientation.w();
    plane.pose.orientation.x = plane_orientation.x();
    plane.pose.orientation.y = plane_orientation.y();
    plane.pose.orientation.z = plane_orientation.z();
    Eigen::Vector3d p_difference((p_2 - p_1) / (p_2 - p_1).norm());
    n.getParam("p_center_x", p_c[0]);
    n.getParam("p_center_y", p_c[1]);
    n.getParam("p_center_z", p_c[2]);
    n.getParam("circle_rad", radius);
    n.getParam("circle_rot", R_flat);
    if (R_flat.size() == 9)
    {
      R_circle = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(R_flat.data());
    }
    else
    {
      R_circle = Eigen::Matrix3d::Identity(); // fallback
    }
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

    if (hybrid_mode_list.size() == NUM_MODES)
    {
      if (hybrid_mode_list[LINE_MODE_IDX])
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
      else if (hybrid_mode_list[PLANE_MODE_IDX])
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

        // Set points
        for (int i = 0; i < spline_points.size(); i += 3)
        {
          geometry_msgs::Point p;
          p.x = spline_points[i];
          p.y = spline_points[i + 1];
          p.z = spline_points[i + 2];
          points.points.push_back(p);
        }

        mode_text.text = "SPLINE MODE";
        marker_pub.publish(mode_text);
        marker_pub.publish(spline);
        // Optionally display the user defined points
        // marker_pub.publish(points);
      }
    }
    else
    {
      ROS_INFO("ERROR: hybrid mode list is incorrect size. Expected size %d, got size %d", NUM_MODES, static_cast<int>(hybrid_mode_list.size()));
    }

#endif

    r.sleep();

    f += 0.04;
  }
}