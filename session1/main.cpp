#include <iostream>

#include <Eigen/Dense>
#include "math/math.h"

constexpr double rad_to_deg = 57.2957795;
constexpr double deg_to_rad = 0.0174532925;

/*double deg_to_rad(double degrees)

{
  return degrees * M_PI/180;
}
double rad_to_deg(double radians)
{
  return radians * 180 / M_PI;
*/


bool floatEquals(double a, double b)
    {
  return std::abs(a-b) < 1e-6;
  }


//--------------------------------------------------
Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d &v)
{
  Eigen::Matrix3d skew_symetric;
  skew_symetric <<
      0, -v(2), v(1),
  v(2), 0, -v(0),
  -v(1), v(0), 0;
  return skew_symetric;

}




Eigen::Matrix3d rotate_x(double radians)
{
  Eigen::Matrix3d matrix;;
  matrix <<
      1.0, 0.0, 0.0,
      0, std::cos(radians),-std::sin(radians),
      0, std::sin(radians), std::cos(radians);
  return matrix;
}


//c) rotational matrix around axis y
Eigen::Matrix3d rotate_y(double radians)
{
  Eigen::Matrix3d matrix;
  matrix <<
      std::cos(radians), 0, std::sin(radians),
     0, 1, 0,
     -std::sin(radians), 0, std::cos(radians);
  return matrix;
}


//d) rotational matrix around axis z

Eigen::Matrix3d rotate_z(double radians)
{
  Eigen::Matrix3d matrix;
  matrix <<
      std::cos(radians), -std::sin(radians), 0,
     std::sin(radians), std::cos(radians), 0,
     0, 0, 1;
  return matrix;
}

//E) matrix rotating around arbitrary axis

Eigen::Matrix3d rotation_matrix_from_axis_angle(const Eigen::Vector3d &axis, double radians)

{

  Eigen::Matrix3d skew = skew_symmetric(axis);
  Eigen::Matrix3d matrix_i = Eigen::Matrix3d::Identity();
  return matrix_i + std::sin(radians)*skew+(1 - std::cos(radians))*skew*skew;
}

// f) rotation matrix from euler angles ZYX

Eigen::Matrix3d rotation_matrix_from_euler_zyx(const Eigen::Vector3d &e)
{
  Eigen::Matrix3d matrix;
  Eigen::Matrix3d ROT_z  = rotate_z(e(0)); //e(0)=alpha
  Eigen::Matrix3d ROT_y  = rotate_y(e(1)); // e(1)=beta
  Eigen::Matrix3d ROT_x  = rotate_x(e(2)); // e(2)=gamma
  matrix = ROT_z * ROT_y * ROT_x;
  return matrix;
}

//-------------------------------------------------------------------------

Eigen::Vector3d euler_zyx_from_rotation(const Eigen::Matrix3d &r)
{
  double a = 0.0;
  double b = 0.0;
  double c = 0.0;

  if(floatEquals(r(2, 0), -1.0))
  {
    b = EIGEN_PI / 2.0;
    a = 0.0;
    c = std::atan2(r(0, 1), r(1, 1));
  }
  else if(floatEquals(r(2, 0), 1.0))
  {
    b = -(EIGEN_PI / 2.0);
    a = 0.0;
    c = -std::atan2(r(0, 1), r(1, 1));
  }
  else
  {
    b = std::atan2(-r(2, 0), std::sqrt(r(0, 0) * r(0, 0) + r(1, 0) * r(1, 0)));
    a = std::atan2(r(1, 0), r(0, 0));
    c = std::atan2(r(2, 1), r(2, 2));
  }
  return Eigen::Vector3d{a, b, c};
}

int main()
{
  Eigen::Vector3d e = Eigen::Vector3d{60.0, 45.0, 30.0} * deg_to_rad;
  Eigen::Matrix3d r = rotation_matrix_from_euler_zyx(e);
  Eigen::Vector3d ea = euler_zyx_from_rotation(r);
  std::cout << " E: " << e.transpose() * rad_to_deg << std::endl;
  std::cout << "Ea: " << ea.transpose() * rad_to_deg << std::endl;
  return 0;
}

//gjort i time 05.09 ---------------------------------------------------------