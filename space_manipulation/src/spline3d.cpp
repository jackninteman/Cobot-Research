#include "space_manipulation/spline3d.h"

Spline3d::Spline3d(Eigen::Vector3d p_1, Eigen::Vector3d p_2, Eigen::Vector3d p_3, Eigen::Vector3d p_4)
{
    // Define the sample points for interpolation
    std::vector<Eigen::Vector3d> sample_points = {
        p_1,
        p_2,
        p_3,
        p_4};

    num_points = sample_points.size();

    Eigen::Matrix<double, 3, Eigen::Dynamic> points(3, num_points);
    for (int i = 0; i < num_points; ++i)
    {
        points.col(i) = sample_points[i];
    }

    spline = Eigen::SplineFitting<Eigen::Spline<double, 3>>::Interpolate(points, 3);
}

void Spline3d::FindDesiredSplinePoint(Eigen::Vector3d current_position)
{
    Eigen::Vector3d p = current_position;

    double min_dist = std::numeric_limits<double>::max();
    double bestT = 0.0;
    Eigen::Vector3d s;

    for (double t = 0; t <= 1; t += 0.001)
    {
        s = spline(t);
        // tangent = spline.derivatives(t, 1).col(1);

        double dist = sqrt(pow(p.x() - s.x(), 2) + pow(p.y() - s.y(), 2) + pow(p.z() - s.z(), 2));

        if (dist < min_dist)
        {
            min_dist = dist;
            bestSplinePoint = s;
            bestT = t;
            // bestTangent = tangent;
        }
    }

    bestTangent = spline.derivatives(bestT, 1).col(1);
}

Eigen::Vector3d Spline3d::GetBestPoint()
{
    return bestSplinePoint;
}

Eigen::Vector3d Spline3d::GetBestTangent()
{
    return bestTangent.normalized();
}

visualization_msgs::Marker Spline3d::GetSplineVis()
{
    this->PrepToPub();
    return spline_strip;
}

void Spline3d::PrepToPub()
{
    // prepare variables for publishing
    spline_strip.header.frame_id = "world";
    spline_strip.header.stamp = ros::Time::now();
    spline_strip.ns = "points_and_lines";
    spline_strip.id = 0;
    spline_strip.type = visualization_msgs::Marker::LINE_STRIP;
    spline_strip.action = visualization_msgs::Marker::ADD;
    spline_strip.scale.x = 0.01; // Line width
    spline_strip.color.r = 1.0;  // spline is pink
    spline_strip.color.g = 0.0;
    spline_strip.color.b = 1.0;
    spline_strip.color.a = 1.0;

    spline_strip.points.clear();
    for (double t = 0; t <= 1; t += 0.001)
    {
        Eigen::Vector3d p = spline(t);

        geometry_msgs::Point pt;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
        spline_strip.points.push_back(pt);
    }
}