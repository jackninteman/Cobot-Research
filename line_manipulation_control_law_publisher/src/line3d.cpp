#include "line3d.h"

Eigen::Vector3d Line3d::GetDesiredCrosstrackLocation(Eigen::Vector3d current_cartesian_position)
{
    //Extract current cartesian position
    double x0 = current_cartesian_position[0];
    double y0 = current_cartesian_position[1];
    double z0 = current_cartesian_position[2];

    //Compute point in line to return to in case of linear line
    if (_x_param[0] == 0 && _y_param[0] == 0 && _z_param[0] == 0)
    {
        double A = std::pow(_x_param[1],2) + std::pow(_y_param[1],2) + std::pow(_z_param[1],2);
        double D = _x_param[1]*(x0 - _x_param[2]) +
                   _y_param[1]*(y0 - _y_param[2]) +
                   _z_param[1]*(z0 - _z_param[2]);
        double t = D/A;
        Eigen::Vector3d point_in_line(_x_param[1]*t + _x_param[2], 
                                      _y_param[1]*t + _y_param[2],
                                      _z_param[1]*t + _z_param[2]);
        return point_in_line;
    }
    else
    {
        //Set up parameters solve for polynomial roots
        double A = -2*(std::pow(_x_param[0],2) + std::pow(_y_param[0],2) + std::pow(_z_param[0],2));
        double B = -3*(_x_param[0]*_x_param[1] + _y_param[0]*_y_param[1] + _z_param[0]*_z_param[1]);
        double C = 2*_x_param[0]*(x0 - _x_param[2]) - std::pow(_x_param[1],2) +
                   2*_y_param[0]*(y0 - _y_param[2]) - std::pow(_y_param[1],2) + 
                   2*_z_param[0]*(z0 - _z_param[2]) - std::pow(_z_param[1],2);
        double D = _x_param[1]*(x0 - _x_param[2]) +
                   _y_param[1]*(y0 - _y_param[2]) +
                   _z_param[1]*(z0 - _z_param[2]);

        //Use ecl_geometry library to solve for 3rd order polynomial roots
        ecl::Polynomial<3> p;
        p.coefficients() << D, C, B, A; // in order D + Ct + Bt^2 + At^3
        std::cout << p << std::endl;
        ecl::Array<double> X = ecl::CubicPolynomial::Roots(p);
        std::cout << X << std::endl;

        //Sometimes it produces NaN solution so we need to get rid of it
        while (std::isnan(X.back()))
        {
            X.resize(X.size()-1);
        }
        double t = this->GetOptimalLineParam(X, current_cartesian_position);
        Eigen::Vector3d optimal_point_in_line(
                            _x_param[0]*std::pow(t,2) + _x_param[1]*t + _x_param[2],
                            _y_param[0]*std::pow(t,2) + _y_param[1]*t + _y_param[2],
                            _z_param[0]*std::pow(t,2) + _z_param[1]*t + _z_param[2]);
        return optimal_point_in_line;
        }
}

double Line3d::GetOptimalLineParam(ecl::Array<double> X, Eigen::Vector3d current_cartesian_position)
{
    std::vector<double> optimal_param_vec;
    double shortest_distance_update;
    for (double t : X)
    {
        Eigen::Vector3d point_in_line(_x_param[0]*std::pow(t,2) + _x_param[1]*t + _x_param[2],
                                      _y_param[0]*std::pow(t,2) + _y_param[1]*t + _y_param[2],
                                      _z_param[0]*std::pow(t,2) + _z_param[1]*t + _z_param[2]);
        if (optimal_param_vec.empty())
        {
            optimal_param_vec.emplace_back(t);
            shortest_distance_update = this->GetDistance(point_in_line, current_cartesian_position);
        }
        else if (this->GetDistance(point_in_line, current_cartesian_position) < shortest_distance_update)
        {
            optimal_param_vec.clear();
            optimal_param_vec.emplace_back(t);
            shortest_distance_update = this->GetDistance(point_in_line, current_cartesian_position);
        }    
    }
    return optimal_param_vec.back();
}

double Line3d::GetDistance(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const
{
    return std::sqrt(a.dot(b));
}