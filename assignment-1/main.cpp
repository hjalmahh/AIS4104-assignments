#include <iostream>

#include <Eigen/Dense>
#include <math.h>

double deg_to_rad(double degrees)

{
    return degrees * M_PI/180;
}
double rad_to_deg(double radians)
{
    return radians * 180 / M_PI;
}



Eigen::Matrix3d rotate_x(double angle)
{
    Eigen::Matrix3d matrix;
    matrix <<
        1.0, 0.0, 0.0,
        0.0, std::cos(angle), -std::sin(angle),
        0.0, std::sin(angle), std::cos(angle);
    return matrix;
}


void example(double constant)
{
    Eigen::Matrix3d identity;
    identity <<
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0;
    std::cout << "I: " << std::endl << identity << std::endl << std::endl;
    std::cout << constant <<"*I: " << std::endl << constant * identity << std::endl << std::endl;
}

int main()
{
    //example(2.0);


    return 0;
}
