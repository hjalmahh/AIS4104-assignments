#include <iostream>
#include <Eigen/Dense>
#include <cmath>



 double deg_to_rad(double degrees)

{
    return degrees * 0.0174533;
}
double rad_to_deg(double radians)
{
    return radians * 57.2958;
}

/*

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
    example(2.0);


    return 0;
} */

// koden for assigemnt starter her

//task 2.1

//Funksjon som regner ut ''skew-symmetric'' matrice fra en vektor

Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d skew_symetric;
    skew_symetric <<
        0, -v(2), v(1),
    v(2), 0, -v(0),
    -v(1), v(0), 0;
    return skew_symetric;

}

void skew_symmetric_test()
{
    Eigen::Matrix3d skew_matrix = skew_symmetric(Eigen::Vector3d{0.5, 0.5, 0.707107});
    std::cout << "Skew-symmetric matrix: " << std::endl;
    std::cout << skew_matrix << std::endl;
    std::cout << "Skew-symmetric matrix transposition: " << std::endl;
    std::cout << -skew_matrix.transpose() << std::endl;
}

/*
int main() {
    skew_symmetric_test();
    return 0;
}*/

// task 2.2

// a) Fnction for rotation matrix giving refrence by principle axes

Eigen::Matrix3d rotation_matrix_from_frame_axes(const Eigen::Vector3d &x,
const Eigen::Vector3d &y,
const Eigen::Vector3d &z)
{
    Eigen::Matrix3d matrix;
    matrix.col(0) = x;
    matrix.col(1) = y;
    matrix.col(2) = z;
    return matrix;
}


//b) rotational matrix around axis x

Eigen::Matrix3d rotate_x(double degrees)
{
     double radians = deg_to_rad(degrees);
    Eigen::Matrix3d matrix;;
    matrix <<
        1.0, 0.0, 0.0,
        0, std::cos(radians),-std::sin(radians),
        0, std::sin(radians), std::cos(radians);
    return matrix;
}


//c) rotational matrix around axis y
Eigen::Matrix3d rotate_y(double degrees)
 {
     double radians = deg_to_rad(degrees);
     Eigen::Matrix3d matrix;
     matrix <<
         std::cos(radians), 0, std::sin(radians),
        0, 1, 0,
        -std::sin(radians), 0, std::cos(radians);
     return matrix;
 }


//d) rotational matrix around axis z

Eigen::Matrix3d rotate_z(double degrees)
 {
     double radians = deg_to_rad(degrees);
     Eigen::Matrix3d matrix;
     matrix <<
         std::cos(radians), -std::sin(radians), 0,
        std::sin(radians), std::cos(radians), 0,
        0, 0, 1;
     return matrix;
 }

//E) matrix rotating around arbitrary axis

Eigen::Matrix3d rotation_matrix_from_axis_angle(const Eigen::Vector3d &axis, double degrees)

{
double radians = deg_to_rad(degrees);
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

void rotation_matrix_test()
{
Eigen::Matrix3d rot =
rotation_matrix_from_euler_zyx(Eigen::Vector3d{45.0, -45.0, 90.0});
Eigen::Matrix3d rot_aa =
rotation_matrix_from_axis_angle(Eigen::Vector3d{0.8164966, 0.0, 0.5773503}, 120.0);
Eigen::Matrix3d rot_fa =
rotation_matrix_from_frame_axes(Eigen::Vector3d{0.5, 0.5, 0.707107},
Eigen::Vector3d{-0.5, -0.5, 0.707107},
Eigen::Vector3d{0.707107, -0.707107, 0.0});
std::cout << "Rotation matrix from Euler: " << std::endl;
std::cout << rot << std::endl << std::endl;
std::cout << "Rotation matrix from axis-angle pair: " << std::endl;
std::cout << rot_aa << std::endl << std::endl;
std::cout << "Rotation matrix from frame axes: " << std::endl;
std::cout << rot_fa << std::endl << std::endl;
}


/*
int main() {
//skew_symmetric_test();
rotation_matrix_test();
    return 0;
}
*/

// task 2.3

Eigen::Matrix4d transformation_matrix(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
{
Eigen::Matrix4d matrix;
//Creating the matrix part
matrix.block<3, 3>(0, 0) = r;
//crating the points for the  matrix from row 1 colum 4 (row zero og kolonne 3 på koding)
matrix.block<3, 1>(0, 3) = p;
//Creating zero row under matrix
matrix.block<1, 3>(3, 0) = Eigen::Vector3d::Zero().transpose();
//oner in bottom rigth
matrix(3, 3) = 1;

return matrix;
}

void transformation_matrix_test()
{
Eigen::Matrix3d r = rotation_matrix_from_euler_zyx(Eigen::Vector3d{45, -45.0, 90.0});
Eigen::Vector3d v{1.0, -2.0, 3.0};
std::cout << "transformation_matrix: " << std::endl;
std::cout << transformation_matrix(r, v) << std::endl;
}




//3.2

void transform_vector()
{
//angles wich alfa is rotaed around
Eigen::Vector3d euler_angles(60, 45,0);
// Rotation matrix from euler angles
Eigen::Matrix3d rot = rotation_matrix_from_euler_zyx(euler_angles);
//Vector for location of rotaed matrix
Eigen::Vector3d p(0,0,10);
//Trasnformation matrix for rotation(rot) and vector(p), (homogenus trasnformation)
Eigen::Matrix4d T = transformation_matrix(rot, p);
// vector exxpressed in alpha frame
Eigen::Vector3d v_a (2.5, 3.0, -10);
//Transform for alpha frame to omega frame
Eigen::Vector4d v_a_h;
v_a_h <<
v_a, 1;
Eigen::Vector4d v_omega_h = T * v_a_h;
//Print
Eigen::Vector3d v_omega = v_omega_h.head<3>();
std::cout << "transformed vector from alpha frame to omega:" << std::endl;
std::cout << v_omega << std::endl;

}


int main() {
skew_symmetric_test();
rotation_matrix_test();
transformation_matrix_test();
transform_vector();
    return 0;
}