#include "space_manipulation/plane3d.h"

Plane3d::Plane3d(std::vector<Eigen::Vector3d> sample_points)
{
    num_points = sample_points.size();

    Eigen::Matrix<double, 2, Eigen::Dynamic> points(2, num_points);
    for (int i = 0; i < num_points; ++i)
    {
        Eigen::Vector2d vec(sample_points[i].y(), sample_points[i].z());
        points.col(i) = vec;
    }

    plane_3d = Eigen::SplineFitting<Eigen::Spline<double, 2>>::Interpolate(points, 2);
}

void Plane3d::FindDesired3dPlanePoint(Eigen::Vector3d current_position, Eigen::Matrix<double, 4, 4> T)
{
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Vector3d p = T.block<3, 1>(0, 3);
    // ROS_INFO_STREAM(T);

    Eigen::Matrix4d Tinv = Eigen::Matrix4d::Identity();
    Tinv.block<3, 3>(0, 0) = R.transpose();
    Tinv.block<3, 1>(0, 3) = -R.transpose() * p;
    // ROS_INFO_STREAM(Tinv);

    Eigen::Vector4d current_position4d(current_position.x(), current_position.y(), current_position.z(), 1.0);
    Eigen::Vector4d current_position_plane4d = Tinv * current_position4d;
    Eigen::Vector3d current_position_plane(current_position_plane4d(0), current_position_plane4d(1), current_position_plane4d(2));
    Eigen::Vector2d current_position_plane2d(current_position_plane.y(), current_position_plane.z());

    auto f = [&](double t)
    {
        Eigen::Vector2d p = plane_3d(t);
        return (p - current_position_plane2d).squaredNorm();
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
    Eigen::Vector2d bestYZ = plane_3d(bestT);
    double bestX = current_position_plane.x();

    Eigen::Vector4d bestPoint4d_plane(bestX, bestYZ.x(), bestYZ.y(), 1.0);
    Eigen::Vector4d bestPoint4d = T * bestPoint4d_plane;

    bestPoint.x() = bestPoint4d(0);
    bestPoint.y() = bestPoint4d(1);
    bestPoint.z() = bestPoint4d(2);
    Eigen::Vector2d bestTangentPlane2d = plane_3d.derivatives(bestT, 1).col(1);
    Eigen::Vector3d bestTangentPlane3d(0, bestTangentPlane2d.x(), bestTangentPlane2d.y());

    bestTangent = R * bestTangentPlane3d;
}

Eigen::Vector3d Plane3d::GetBestPoint()
{
    return bestPoint;
}

Eigen::Vector3d Plane3d::GetBestTangent()
{
    return bestTangent.normalized();
}

visualization_msgs::Marker Plane3d::Get3dPlaneVis(Eigen::Matrix<double, 4, 4> T)
{
    this->PrepToPub(T);
    return plane_3d_strip;
}

void Plane3d::PrepToPub(Eigen::Matrix<double, 4, 4> T)
{
    // prepare variables for publishing
    plane_3d_strip.header.frame_id = "panda_link0";
    plane_3d_strip.header.stamp = ros::Time::now();
    plane_3d_strip.ns = "points_and_lines";
    plane_3d_strip.id = 0;
    plane_3d_strip.type = visualization_msgs::Marker::TRIANGLE_LIST;
    plane_3d_strip.action = visualization_msgs::Marker::ADD;
    plane_3d_strip.pose.orientation.w = 1.0;
    // plane_3d_strip.scale.x = 0.05; // Line width
    plane_3d_strip.scale.x = 1.0;
    plane_3d_strip.scale.y = 1.0;
    plane_3d_strip.scale.z = 1.0;
    plane_3d_strip.color.r = 0.0; // spline is pink
    plane_3d_strip.color.g = 0.7;
    plane_3d_strip.color.b = 0.0;
    plane_3d_strip.color.a = 0.8;

    int Nx = 30;
    int Nu = 100;
    double length = 0.5;

    std::vector<std::vector<Eigen::Vector3d>> grid_neg(Nx, std::vector<Eigen::Vector3d>(Nu));
    std::vector<std::vector<Eigen::Vector3d>> grid_pos(Nx, std::vector<Eigen::Vector3d>(Nu));

    for (int ix = 0; ix < Nx; ix++)
    {
        double x = (double)ix / (Nx - 1) * length;

        for (int iu = 0; iu < Nu; iu++)
        {
            double u = (double)iu / (Nu - 1);

            Eigen::Vector2d yz = plane_3d(u);
            Eigen::Vector4d vec4d(x, yz(0), yz(1), 1.0);
            Eigen::Vector4d newvec4d = T * vec4d;
            grid_pos[ix][iu] = Eigen::Vector3d(newvec4d(0), newvec4d(1), newvec4d(2));
        }
    }

    for (int ix = 0; ix < Nx; ix++)
    {
        double x = -(double)ix / (Nx - 1) * length;

        for (int iu = 0; iu < Nu; iu++)
        {
            double u = (double)iu / (Nu - 1);

            Eigen::Vector2d yz = plane_3d(u);
            Eigen::Vector4d vec4d(x, yz(0), yz(1), 1.0);
            Eigen::Vector4d newvec4d = T * vec4d;
            grid_neg[ix][iu] = Eigen::Vector3d(newvec4d(0), newvec4d(1), newvec4d(2));
        }
    }

    for (int ix = 0; ix < Nx - 1; ix++)
    {
        for (int iu = 0; iu < Nu - 1; iu++)
        {

            Eigen::Vector3d p00 = grid_pos[ix][iu];
            Eigen::Vector3d p01 = grid_pos[ix][iu + 1];
            Eigen::Vector3d p10 = grid_pos[ix + 1][iu];
            Eigen::Vector3d p11 = grid_pos[ix + 1][iu + 1];

            // Triangle 1: p00, p10, p11
            geometry_msgs::Point a, b, c;
            a.x = p00.x();
            a.y = p00.y();
            a.z = p00.z();
            b.x = p10.x();
            b.y = p10.y();
            b.z = p10.z();
            c.x = p11.x();
            c.y = p11.y();
            c.z = p11.z();
            plane_3d_strip.points.push_back(a);
            plane_3d_strip.points.push_back(b);
            plane_3d_strip.points.push_back(c);

            // Triangle 2: p00, p11, p01
            geometry_msgs::Point d, e, f;
            d.x = p00.x();
            d.y = p00.y();
            d.z = p00.z();
            e.x = p11.x();
            e.y = p11.y();
            e.z = p11.z();
            f.x = p01.x();
            f.y = p01.y();
            f.z = p01.z();
            plane_3d_strip.points.push_back(d);
            plane_3d_strip.points.push_back(e);
            plane_3d_strip.points.push_back(f);
        }
    }

    for (int ix = 0; ix < Nx - 1; ix++)
    {
        for (int iu = 0; iu < Nu - 1; iu++)
        {

            Eigen::Vector3d p00 = grid_neg[ix][iu];
            Eigen::Vector3d p01 = grid_neg[ix][iu + 1];
            Eigen::Vector3d p10 = grid_neg[ix + 1][iu];
            Eigen::Vector3d p11 = grid_neg[ix + 1][iu + 1];

            // Triangle 1: p00, p10, p11
            geometry_msgs::Point a, b, c;
            a.x = p00.x();
            a.y = p00.y();
            a.z = p00.z();
            b.x = p11.x();
            b.y = p11.y();
            b.z = p11.z();
            c.x = p10.x();
            c.y = p10.y();
            c.z = p10.z();
            plane_3d_strip.points.push_back(a);
            plane_3d_strip.points.push_back(b);
            plane_3d_strip.points.push_back(c);

            // Triangle 2: p00, p11, p01
            geometry_msgs::Point d, e, f;
            d.x = p00.x();
            d.y = p00.y();
            d.z = p00.z();
            e.x = p01.x();
            e.y = p01.y();
            e.z = p01.z();
            f.x = p11.x();
            f.y = p11.y();
            f.z = p11.z();
            plane_3d_strip.points.push_back(d);
            plane_3d_strip.points.push_back(e);
            plane_3d_strip.points.push_back(f);
        }
    }
}