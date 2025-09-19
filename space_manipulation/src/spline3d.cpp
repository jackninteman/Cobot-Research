#include "space_manipulation/spline3d.h"

Spline3d::Spline3d(std::vector<Eigen::Vector3d> sample_points)
{
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
    auto f = [&](double t)
    {
        Eigen::Vector3d p = spline(t);
        return (p - current_position).squaredNorm();
    };

    const double phi = 0.6180339887498949;
    double a = 0, b = 1;
    // compute initial guesses using phi
    double c = b - phi * (b - a);
    double d = a + phi * (b - a);

    // continuously update guesses until the difference is within the specified tolerance
    double tol = 1e-6;
    while (std::abs(c - d) > tol)
    {
        if (f(c) < f(d))
            b = d;
        else
            a = c;

        c = b - phi * (b - a);
        d = a + phi * (b - a);
    }

    // Use the calculated "best t" to compute position and derivative of the spline
    double bestT = 0.5 * (a + b);
    bestSplinePoint = spline(bestT);
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