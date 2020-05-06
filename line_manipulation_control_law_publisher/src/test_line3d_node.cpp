#include <iostream>
#include <vector>
#include "line3d.h"

int main()
{
    std::vector<double> a = {0,1,0};
    std::vector<double> b = {0,0,0};
    std::vector<double> c = {0,0,0.12};
    Line3d line(a,b,c);
    Eigen::Vector3d current_cartesian_position(0.1,0,0);
    Eigen::Vector3d point_in_line;
    point_in_line << line.GetDesiredCrosstrackLocation(current_cartesian_position);
    std::cout << "Point Solution is: " << std::endl << point_in_line << std::endl;
    return 0;
}