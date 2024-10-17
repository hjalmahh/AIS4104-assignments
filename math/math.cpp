#include <iostream>

#include <Eigen/Dense>

#include "math/math.h"

#include <numeric>


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


bool math::floatEquals(double a, double b)
{
    return std::abs(a - b) < 1e-6;
}


//--------------------------------------------------
Eigen::Matrix3d math::skew_symmetric(const Eigen::Vector3d& v)
{
    Eigen::Matrix3d skew_symetric;
    skew_symetric <<
        0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return skew_symetric;
}

Eigen::Vector3d math::vector_from_skew_symmetric(const Eigen::Matrix3d& v)
{
    return Eigen::Vector3d(v(2, 1), v(0, 2), v(1, 0));
}


Eigen::Matrix3d math::rotate_x(double radians)
{
    Eigen::Matrix3d matrix;;
    matrix <<
        1.0, 0.0, 0.0,
        0.0, std::cos(radians), -std::sin(radians),
        0.0, std::sin(radians), std::cos(radians);
    return matrix;
}


//c) rotational matrix around axis y
Eigen::Matrix3d math::rotate_y(double radians)
{
    Eigen::Matrix3d matrix;
    matrix <<
        std::cos(radians), 0.0, std::sin(radians),
        0.0, 1, 0.0,
        -std::sin(radians), 0.0, std::cos(radians);
    return matrix;
}


//d) rotational matrix around axis z

Eigen::Matrix3d math::rotate_z(double radians)
{
    Eigen::Matrix3d matrix;
    matrix <<
        std::cos(radians), -std::sin(radians), 0.0,
        std::sin(radians), std::cos(radians), 0.0,
        0.0, 0.0, 1;
    return matrix;
}

//E) matrix rotating around arbitrary axis

Eigen::Matrix3d math::rotation_matrix_from_axis_angle(const Eigen::Vector3d& axis, double radians)

{
    Eigen::Matrix3d skew = skew_symmetric(axis);
    Eigen::Matrix3d matrix_i = Eigen::Matrix3d::Identity();
    return matrix_i + std::sin(radians) * skew + (1 - std::cos(radians)) * skew * skew;
}

// f) rotation matrix from euler angles ZYX

Eigen::Matrix3d math::rotation_matrix_from_euler_zyx(const Eigen::Vector3d& e)
{
    Eigen::Matrix3d matrix;
    Eigen::Matrix3d ROT_z = rotate_z(e(0)); //e(0)=alpha
    Eigen::Matrix3d ROT_y = rotate_y(e(1)); // e(1)=beta
    Eigen::Matrix3d ROT_x = rotate_x(e(2)); // e(2)=gamma
    matrix = ROT_z * ROT_y * ROT_x;
    return matrix;
}

Eigen::Matrix4d math::transformation_matrix(const Eigen::Matrix3d& r, const Eigen::Vector3d& p)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = r;
    T.block<3, 1>(0, 3) = p;
    return T;
}

//-------------------------------------------------------------------------

Eigen::Vector3d math::euler_zyx_from_rotation(const Eigen::Matrix3d& r)
{
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;

    if (math::floatEquals(r(2, 0), -1.0))
    {
        b = EIGEN_PI / 2.0;
        a = 0.0;
        c = std::atan2(r(0, 1), r(1, 1));
    }
    else if (math::floatEquals(r(2, 0), 1.0))
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


//gjort i time 05.09 ---------------------------------------------------------


//asigment 2 ---------------------------------


// oppgave b assigment 2
Eigen::VectorXd math::twist(const Eigen::Vector3d& W, const Eigen::Vector3d& V)
{
    Eigen::VectorXd twist(6);
    twist << W(0), W(1), W(2), V(0), V(1), V(2);
    return twist;
}


//oppgave c

Eigen::VectorXd math::screw_axis(const Eigen::Vector3d& q, const Eigen::Vector3d& s, double h)
{
    // Initialize the screw axis (6x1 vector)
    Eigen::VectorXd screw_axis(6);
    Eigen::Vector3d v = -s.cross(q) + h * s;

    screw_axis << s, v;

    return screw_axis;
}

//oppgave d
Eigen::MatrixXd math::adjoint_matrix(const Eigen::Matrix4d& tf)
{
    Eigen::MatrixXd adt(6, 6);
    // Picks R & p from  T matrix
    Eigen::Matrix3d R = tf.block<3, 3>(0, 0);
    Eigen::Vector3d p = tf.block<3, 1>(0, 3);
    //define null matrix
    Eigen::Matrix3d nullmatrix = Eigen::Matrix3d::Zero();
    // construct adt from rotation matrix and transdlation vector
    //adt << R, nullmatrix, skew_symmetric(p) * R, R;
    //''strengt bestemer posisjoner''
    adt.block<3, 3>(0, 0) = R;
    adt.block<3, 3>(0, 3) = nullmatrix;
    adt.block<3, 3>(3, 0) = skew_symmetric(p) * R;
    adt.block<3, 3>(3, 3) = R;

    return adt;
}


double math::cot(double x)
{
    return 1.0 / std::tan(x);
}

//task 2a --------------------

Eigen::Matrix3d math::rotation_matrix_from_euler_yzx(const Eigen::Vector3d& e)
{
    Eigen::Matrix3d matrix;
    //e(0)=alpha, e(1)=beta, e(2)=gamma
    Eigen::Matrix3d ROT_z = rotate_z(e(1));
    Eigen::Matrix3d ROT_y = rotate_y(e(0));
    Eigen::Matrix3d ROT_x = rotate_x(e(2));
    matrix = ROT_y * ROT_z * ROT_x;
    return matrix;
}

//task 2b example 3.28 modern robotics

Eigen::VectorXd math::wrench_sum()
{
    //vector and matrixes
    Eigen::VectorXd ff(6);
    Eigen::VectorXd fh(6);
    Eigen::VectorXd fa(6);
    Eigen::Matrix4d thf;
    Eigen::Matrix4d taf;
    //vector and matrix values from example
    fh << 0, 0, 0, 0, -5, 0;
    fa << 0, 0, 0, 0, 0, 1;
    thf << 1, 0, 0, -0.1,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    taf << 1, 0, 0, -0.25,
        0, 0, 1, 0,
        0, -1, 0, 0,
        0, 0, 0, 1;
    ff = adjoint_matrix(thf).transpose() * fh + adjoint_matrix(taf).transpose() * fa;
    return ff;
}

//task 3 a  Implement the matrix exponential for rotation matrices.

Eigen::Matrix3d math::matrix_exponential(const Eigen::Vector3d& w, double theta)
{
    Eigen::Matrix3d skew_w{skew_symmetric(w)};
    Eigen::Matrix3d matrix_i = Eigen::Matrix3d::Identity();
    const double radians = theta * deg_to_rad;

    return matrix_i + std::sin(radians) * skew_w + (1.0 - std::cos(radians)) * skew_w * skew_w;
}




//task 3b Implement the matrix logarithm for rotation matrices.

std::pair<Eigen::Vector3d, double> math::matrix_logarithm(const Eigen::Matrix3d& r)
{
    double theta = 0.0;
    Eigen::Vector3d w = Eigen::Vector3d::Zero();
    if (math::floatEquals(r.trace(), -1.0))
    {
        theta = EIGEN_PI;
        if (!math::floatEquals(r(2, 2), -1.0))
        {
            w = 1 / (std::sqrt(2 * (1 + (r(2, 2))))) * Eigen::Vector3d(r(0, 2), r(1, 2), 1.0 + r(2, 2));
        }
        else if (!math::floatEquals(r(1, 1), -1.0))
        {
            w = 1 / (std::sqrt(2 * (1 + (r(2, 2))))) * Eigen::Vector3d(r(0, 1), 1 + r(1, 1), r(2, 1));
        }
        else if (!math::floatEquals(r(0, 0), -1.0))
        {
            w = 1 / (std::sqrt(2 * (1 + (r(2, 2))))) * Eigen::Vector3d(1 + r(0, 0), r(1, 0), r(2, 0));
        }
    }
    else if (!r.isIdentity())
    {
        theta = std::acos(0.5 * (r.trace() - 1));
        w = vector_from_skew_symmetric(1 / (2 * std::sin(theta)) * (r - r.transpose()));
    }
    return std::make_pair(w, theta);

    /*Eigen::AngleAxisd aa(r); //gir akse og rotasjon basert på rotasjons matrise
    Eigen::Vector3d rot_vector = aa.axis() * aa.angle(); //gir rotasjonsvector
    double theta = aa.angle();

    return std::make_pair(rot_vector, theta);*/
}

//task 3c Implement the matrix exponential for homogeneous transformation matrices.

Eigen::Matrix4d math::matrix_exponential(const Eigen::Vector3d& w, const Eigen::Vector3d& v, double theta)
{
    const double radians = theta * deg_to_rad;
    Eigen::Matrix4d T = Eigen::Matrix4d::Zero();
    if (floatEquals(w.norm(), 1.0))
    {
        const Eigen::Matrix3d skew_w{skew_symmetric(w)};
        const Eigen::Matrix3d matrix_i = Eigen::Matrix3d::Identity();
        const Eigen::Matrix3d rotation_matrix = matrix_exponential(w, theta);


        //Translation of T
        Eigen::Matrix3d pt = matrix_i * radians + (1 - std::cos(radians)) * skew_w + (radians - std::sin(radians)) *
            skew_w * skew_w;

        Eigen::Vector3d p = pt * v;

        //Creating the 4x4 transformation matrix
        T = Eigen::Matrix4d::Identity(); // Initialize as identity matrix
        T.block<3, 3>(0, 0) = rotation_matrix; // Top-left 3x3 block is the rotation matrix
        T.block<3, 1>(0, 3) = p; // Top-right 3x1 block is the translation vector
    }

    else if (math::floatEquals(v.norm(), 1.0) && math::floatEquals(w.norm(), 0.0))
    {
        T = Eigen::Matrix4d::Identity();
        T.block<3, 1>(0, 3) = v * radians;
    }
    return T;
}

// 3d Implement the matrix logarithm for homogeneous transformation matrices
Eigen::Matrix3d math::G(const Eigen::Vector3d& w, const double& theta)
{
    Eigen::Matrix3d matrix_i = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d skew_w = skew_symmetric(w);
    const double radians = theta * deg_to_rad;
    ///DOBBELSJEKK RETURN!!!!
    return matrix_i * radians + (1 - std::cos(radians)) * skew_w + (radians - std::sin(radians)) * skew_w * skew_w;
}

Eigen::Matrix3d math::G_inv(const Eigen::Vector3d& w, const double& theta)
{
    Eigen::Matrix3d matrix_i = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d skew_w{skew_symmetric(w)};
    const double radians = theta * deg_to_rad;

    return matrix_i * 1 / radians - 0.5 * skew_w + (1 / radians - 0.5 * cot(radians / 2)) * skew_w * skew_w;
}


std::pair<Eigen::VectorXd, double> math::matrix_logarithm(const Eigen::Matrix4d& t)
{
    Eigen::Matrix3d R = t.block<3, 3>(0, 0); // Top-left 3x3 block is the rotation matrix
    Eigen::Vector3d p = t.block<3, 1>(0, 3); // Top-right 3x1 block is the translation vector

    double theta = 0.0;
    Eigen::Vector3d w = Eigen::Vector3d::Zero();
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    if (R.isIdentity())
    {
        v = p.normalized();
        theta = p.norm();
    }

    else
    {
        std::tie(w, theta) = matrix_logarithm(R);
        v = G_inv(w, theta) * p;
    }
    Eigen::VectorXd s(6);
    s << w, v;

    return std::make_pair(s, theta);
}


//Task 4a

void math::print_pose(const Eigen::Matrix4d& tf, const std::string& label)
{
    //Extract Rotation Matrix and Translation Vector
    Eigen::Matrix3d R = tf.block<3, 3>(0, 0); // Top-left 3x3 block is the rotation matrix
    Eigen::Vector3d p = tf.block<3, 1>(0, 3); // Top-right 3x1 block is the translation vector

    //Rotation matrix to euler angles
    Eigen::Vector3d ZYXeuler = euler_zyx_from_rotation(R) * rad_to_deg;

    if (!label.empty())
        std::cout << label << ":" << std::endl;
    std::cout << "Euler ZYX angles [" << '\n' << ZYXeuler.transpose() << "]\n";
    std::cout << "linear position" << '\n' << p.transpose() << '\n';
}

//task 4b

Eigen::Matrix4d math::planar_3r_fk_transform(const std::vector<double>& joint_positions)
{
    constexpr double l1 = 10, l2 = 10, l3 = 10;
    //Joint positions
    double theta1 = joint_positions[0] * deg_to_rad;
    double theta2 = joint_positions[1] * deg_to_rad;
    double theta3 = joint_positions[2] * deg_to_rad;

    //The homogenus transfer matrix for the joints
    Eigen::Matrix4d T01 = transformation_matrix(rotate_z(theta1), Eigen::Vector3d(0, 0, 0));
    Eigen::Matrix4d T12 = transformation_matrix(rotate_z(theta2), Eigen::Vector3d(l1, 0, 0));
    Eigen::Matrix4d T23 = transformation_matrix(rotate_z(theta3), Eigen::Vector3d(l2, 0, 0));
    Eigen::Matrix4d T34 = transformation_matrix(Eigen::Matrix3d::Identity(), Eigen::Vector3d(l3, 0, 0));

    //end efector pose based on joint angles.
    Eigen::Matrix4d T04 = T01 * T12 * T23 * T34;

    return T04;
}

//task 4c


Eigen::Matrix4d math::planar_3r_fk_screw(const std::vector<double>& joint_positions)
{
    constexpr double l1 = 10, l2 = 10, l3 = 10;
    //joint positions
    double theta1 = joint_positions[0];
    double theta2 = joint_positions[1];
    double theta3 = joint_positions[2];

    Eigen::Vector3d w = {0.0, 0.0, 1.0};
    //skrew axises for the joints
    Eigen::VectorXd s1 = math::screw_axis({0.0, 0.0, 0.0}, w, 0.0);
    Eigen::VectorXd s2 = math::screw_axis({l1, 0.0, 0.0}, w, 0.0);
    Eigen::VectorXd s3 = math::screw_axis({l1 + l2, 0.0, 0.0}, w, 0.0);
    //extrct the liear velocityes
    Eigen::Vector3d v1 = s1.tail(3);
    // Eigen::Vector3d v1 = {s1(3), s1(4), s1(5)};
    Eigen::Vector3d v2 = {s2(3), s2(4), s2(5)};
    Eigen::Vector3d v3 = {s3(3), s3(4), s3(5)};

    //
    Eigen::Matrix4d T01 = math::matrix_exponential(w, v1, theta1);
    Eigen::Matrix4d T12 = math::matrix_exponential(w, v2, theta2);
    Eigen::Matrix4d T23 = math::matrix_exponential(w, v3, theta3);
    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
    M(0, 3) = l1 + l2 + l3;

    Eigen::Matrix4d T04 = T01 * T12 * T23 * M;
    return T04;
}

//task 5: Forward kinematics: UR3e 6R open chain

//5 a

Eigen::Matrix4d math::ur3e_fk_screw(const std::vector<double>& joint_positions)
{
    //assigment values
    constexpr double l1 = 0.24355, l2 = 0.2132;
    constexpr double h1 = 0.15185, h2 = 0.08535;
    constexpr double w1 = 0.13105, w2 = 0.0921;


    //fra bok eksempel
    /*
    constexpr double l1 =0.425, l2 = 0.392;
    constexpr double h1 = 0.089, h2 = 0.095;
    constexpr double w1=0.109, w2=0.082;
    */

    //joit positions
    double theta1 = joint_positions[0];
    double theta2 = joint_positions[1];
    double theta3 = joint_positions[2];
    double theta4 = joint_positions[3];
    double theta5 = joint_positions[4];
    double theta6 = joint_positions[5];

    //skrew axises for the joints
    Eigen::VectorXd s1 = math::screw_axis({0.0, 0.0, 0.0}, {0, 0, 1}, 0.0);
    Eigen::VectorXd s2 = math::screw_axis({0.0, 0.0, h1}, {0, 1, 0}, 0.0);
    Eigen::VectorXd s3 = math::screw_axis({l1, 0.0, h1}, {0, 1, 0}, 0.0);
    Eigen::VectorXd s4 = math::screw_axis({l1 + l2, 0.0, h1}, {0, 1, 0}, 0.0);
    Eigen::VectorXd s5 = math::screw_axis({l1 + l2, w1, h1}, {0, 0, -1}, 0.0);
    Eigen::VectorXd s6 = math::screw_axis({l1 + l2, w1 + w2, h1 - h2}, {0, 1, 0}, 0.0);

    //null config
    Eigen::Matrix4d M;
    M << -1, 0, 0, l1 + l2,
        0, 0, 1, w1 + w2,
        0, 1, 0, h1 - h2,
        0, 0, 0, 1;


    Eigen::Matrix4d T01 = math::matrix_exponential(s1.head(3), s1.tail(3), theta1);
    Eigen::Matrix4d T12 = math::matrix_exponential(s2.head(3), s2.tail(3), theta2);
    Eigen::Matrix4d T23 = math::matrix_exponential(s3.head(3), s3.tail(3), theta3);
    Eigen::Matrix4d T34 = math::matrix_exponential(s4.head(3), s4.tail(3), theta4);
    Eigen::Matrix4d T45 = math::matrix_exponential(s5.head(3), s2.tail(3), theta5);
    Eigen::Matrix4d T56 = math::matrix_exponential(s6.head(3), s6.tail(3), theta6);

    Eigen::Matrix4d T06 = T01 * T12 * T23 * T34 * T45 * T56 * M;

    return T06;
}

//task 5b
Eigen::Matrix4d math::ur3e_fk_transform(const std::vector<double>& joint_positions)
{
    //assigment values
    constexpr double l1 = 0.24355, l2 = 0.2132;
    constexpr double h1 = 0.15185, h2 = 0.08535;
    constexpr double w1 = 0.13105, w2 = 0.0921;


    //fra bok eksempel
    /*
    constexpr double l1 =0.425, l2 = 0.392;
    constexpr double h1 = 0.089, h2 = 0.095;
    constexpr double w1=0.109, w2=0.082;
    */

    //joit positions

    double theta1 = joint_positions[0] * deg_to_rad;
    double theta2 = joint_positions[1] * deg_to_rad;
    double theta3 = joint_positions[2] * deg_to_rad;
    double theta4 = joint_positions[3] * deg_to_rad;
    double theta5 = joint_positions[4] * deg_to_rad;
    double theta6 = joint_positions[5] * deg_to_rad;


    //joit positions
    /*
   double theta1 = joint_positions[0];
    double theta2 = joint_positions[1];
    double theta3= joint_positions[2];
    double theta4= joint_positions[3];
    double theta5= joint_positions[4];
    double theta6= joint_positions[5];
*/

    Eigen::Matrix3d sluttposisjon; // the endeffector frame realtive to last joints frame
    sluttposisjon << -1, 0, 0,
        0, 0, 1,
        0, 1, 0;

    Eigen::Matrix4d T01 = math::transformation_matrix(math::rotate_z(theta1), Eigen::Vector3d(0, 0, 0));
    Eigen::Matrix4d T12 = math::transformation_matrix(math::rotate_y(theta2), Eigen::Vector3d(0, 0, h1));
    Eigen::Matrix4d T23 = math::transformation_matrix(math::rotate_y(theta3), Eigen::Vector3d(l1, 0, 0));
    Eigen::Matrix4d T34 = math::transformation_matrix(math::rotate_y(theta4), Eigen::Vector3d(l2, 0, 0));
    Eigen::Matrix4d T45 = math::transformation_matrix(math::rotate_z(-theta5), Eigen::Vector3d(0, w1, 0));
    Eigen::Matrix4d T56 = math::transformation_matrix(math::rotate_y(theta6), Eigen::Vector3d(0, 0, -h2));
    Eigen::Matrix4d T67 = math::transformation_matrix(sluttposisjon, Eigen::Vector3d(0, w2, 0));

    Eigen::Matrix4d T06 = T01 * T12 * T23 * T34 * T45 * T56 * T67;

    return T06;
}


//-------------------------ASSIGMENT 3----------------------------

//task1 1.a Create a function that converts between vector types

Eigen::VectorXd math::std_vector_to_eigen(const std::vector<double>& v)
{
    Eigen::VectorXd r(v.size());
    for (int i = 0; i < v.size(); i++)
        r[i] = v[i];
    return r;
}

//task 1b  Compare a number to the average value of the last items in a collection
bool math::is_average_below_eps(const std::vector<double>& values, double eps = 10e-7, uint8_t n_values = 5u)
{
    if (values.size() < n_values)
        return false;
    double sum = std::accumulate(values.end() - n_values, values.end(), 0.0);
    return std::abs(sum / n_values) < eps;
}

//task c Construct the UR3e kinematic chain in space frame

std::pair<Eigen::Matrix4d, std::vector<Eigen::VectorXd>> math::ur3e_space_chain()
{
    //assigment values
    constexpr double l1 = 0.24355, l2 = 0.2132;
    constexpr double h1 = 0.15185, h2 = 0.08535;
    constexpr double w1 = 0.13105, w2 = 0.0921;

    //skrew axises for the joints
    std::vector<Eigen::VectorXd> screws{
        math::screw_axis({0.0, 0.0, 0.0}, {0, 0, 1}, 0.0),
        math::screw_axis({0.0, 0.0, h1}, {0, 1, 0}, 0.0),
        math::screw_axis({l1, 0.0, h1}, {0, 1, 0}, 0.0),
        math::screw_axis({l1 + l2, 0.0, h1}, {0, 1, 0}, 0.0),
        math::screw_axis({l1 + l2, w1, h1}, {0, 0, -1}, 0.0),
        math::screw_axis({l1 + l2, w1 + w2, h1 - h2}, {0, 1, 0}, 0.0)
    };
    //null config
    Eigen::Matrix4d m;
    m << -1, 0, 0, l1 + l2,
        0, 0, 1, w1 + w2,
        0, 1, 0, h1 - h2,
        0, 0, 0, 1;

    return std::make_pair(m, screws);
}


Eigen::Matrix4d math::matrix_exponential(const Eigen::VectorXd& screw, double theta)
{
    return matrix_exponential(screw.head<3>(), screw.tail<3>(), theta);
}


//task 1d  Calculate forward kinematics using the UR3e kinematic chain expressed in the space frame
Eigen::Matrix4d math::ur3e_space_fk(const Eigen::VectorXd& joint_positions)
{
    /*
       Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
      std::vector<Eigen::VectorXd> screws;
      std::tie(M, screws) = ur3e_space_chain();
  */
    auto [m, space_screws] = ur3e_space_chain();
    Eigen::Matrix4d t06 = Eigen::Matrix4d::Identity();
    for (int i = 0; i < joint_positions.size(); i++)
        t06 *= math::matrix_exponential(space_screws[i], joint_positions[i]);
    return t06 * m;
}

//task 1 e Construct the UR3e kinematic chain in body frame.
std::pair<Eigen::Matrix4d, std::vector<Eigen::VectorXd>> math::ur3e_body_chain()
{
    auto [m, space_screws] = ur3e_space_chain();

    Eigen::Matrix4d m_inv = m.inverse();

    std::vector<Eigen::VectorXd> body_screws;
    for (const auto& screw : space_screws)
        body_screws.emplace_back(adjoint_matrix(m_inv) * screw);

    return std::make_pair(m, body_screws);
}

// 1f Calculate forward kinematics using the UR3e kinematic chain expressed in the body frame

Eigen::Matrix4d math::ur3e_body_fk(const Eigen::VectorXd& joint_positions)
{
    auto [m, body_screws] = ur3e_body_chain();
    Eigen::Matrix4d t06 = Eigen::Matrix4d::Identity();
    for (int i = 0; i < joint_positions.size(); i++)
        t06 *= math::matrix_exponential(body_screws[i], joint_positions[i]);
    return m * t06;
}


//--------------------task 2----------------------

//task 2 a
std::pair<uint32_t, double> math::newton_raphson_root_find(const std::function<double(double)>& f, double x_0,
                                                           double dx_0, double eps)
{
    int max_iter = 100; //max iterations it wil run
    int iter_count = 0; //counts iterations
    double an = 0; //n'te stigningstall
    double xn = x_0; //current approximation

    while (iter_count < max_iter && (abs(f(xn)) >= eps))
    {
        an = (f(xn + dx_0) - f(xn)) / dx_0;

        if (std::abs(an) < 1e-12)
        {
            std::cout << "Zero derivative found" << std::endl;
            return {iter_count, xn};
        }
        xn = xn - f(xn) / an;

        iter_count++;
    }
    return std::pair(iter_count, xn);
}

//task 2b
//------------------------IKKERIKTIG MÅ ORDNES-----------------

std::pair<uint32_t, double> math::gradient_descent_root_find(const std::function<double(double)>& f, double x_0,
                                                             double gamma, double dx_0, double eps)
{
    int max_iter = 300000; //max iterations it wil run
    int iter_count = 0; //counts iterations
    double an = 0.0; //n'te stigningstall
    double xn = x_0; //current approximation
    double xni = xn + dx_0; //new

    while (iter_count < max_iter && (abs(f(xni)) >= eps))
    {
        an = (abs(f(xni)) - abs(f(xn))) / (xni - xn);
        // if(std::abs(an)< 1e-12)
        if (std::abs(f(xni)) < eps)
        {
            std::cout << "Zero derivative found" << std::endl;
            return {iter_count, xni};
        }

        xn = xni;
        xni = xni - gamma * an;

        iter_count++;
    }
    return std::pair(iter_count, xni);
}

//----------------Task 3-----------------

//task 3a

Eigen::MatrixXd math::ur3e_space_jacobian(const Eigen::VectorXd& current_joint_positions)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    auto [m, screws] = math::ur3e_space_chain();
    const int num_joints = screws.size(); //basere DOF på fuknsjonen som brukes

    //6x6 jacobian matrix
    Eigen::MatrixXd jac_space(6, num_joints);
    jac_space.setZero();

    for (int i = 0; i < num_joints; i++)
    {
        jac_space.col(i) = math::adjoint_matrix(T) * screws[i];

        T = T * math::matrix_exponential(screws[i], current_joint_positions[i]);
    }
    return jac_space;
}


//task 3 b

Eigen::MatrixXd math::ur3e_body_jacobian(const Eigen::VectorXd& current_joint_positions)
{
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        auto [m, screws] = math::ur3e_body_chain();
        const int num_joints = screws.size(); //basere DOF på fuknsjonen som brukes

        //6x6 jacobian matrix
        Eigen::MatrixXd jac_body(6, num_joints);
        jac_body.setZero();

        for (int i = num_joints - 1; i != -1; i--)
        {
            std::cout << "Screw " << i << ": " << screws[i].transpose() << std::endl;
            jac_body.col(i) = math::adjoint_matrix(T) * screws[i];

            T = T * math::matrix_exponential(screws[i], current_joint_positions[i]).inverse();
        }
        return jac_body;
    }
}


//task 4

//4a


std::pair<size_t, Eigen::VectorXd> math::ur3e_ik_body(const Eigen::Matrix4d& t_sd,
                                                      const Eigen::VectorXd& current_joint_positions, double gamma,
                                                      double v_e, double w_e)
{
    int max_iter = 10000; //max iterations it wil run
    int iter_count = 0; //counts iterations
    Eigen::VectorXd v_b(6);

    Eigen::VectorXd theta = current_joint_positions;
    //the configuration of the robot from where to start the numerical solver/algorithm.

    std::pair<Eigen::VectorXd, double> p;

    while ((v_b.head(3).norm() > v_e) || (v_b.tail(3).norm() > w_e))
    {
        Eigen::Matrix4d t_sb = math::ur3e_space_fk(theta);

        Eigen::Matrix4d t_bd = t_sb.inverse() * t_sd;

        p = math::matrix_logarithm(t_bd);
        v_b = p.first * p.second;

        Eigen::MatrixXd jac = math::ur3e_space_jacobian(theta);
        theta += gamma * jac.completeOrthogonalDecomposition().pseudoInverse() * v_b;

        if (iter_count > max_iter)
        {
            return std::make_pair(iter_count, theta);
        }

        iter_count++;
        //std::cout << iter_count << std::endl;
    }
    //psudo invers funksjon CompleteOrthogonalDecomposition
    return std::make_pair(iter_count, theta);
}
