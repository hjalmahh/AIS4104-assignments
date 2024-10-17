//
// Created by hilde on 05.09.2024.
//

#ifndef MATH_H
#define MATH_H

#include<Eigen/Dense>


namespace math{

  constexpr double rad_to_deg = 57.2957795;
  constexpr double deg_to_rad = 0.0174532925;

bool floatEquals(double a, double b);

  Eigen::VectorXd twist(const Eigen::Vector3d &w, const Eigen::Vector3d &v);
Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d &v);
  Eigen::Vector3d vector_from_skew_symmetric(const Eigen::Matrix3d& v);
Eigen::Matrix3d rotate_x(double radians);
Eigen::Matrix3d rotate_y(double radians);
Eigen::Matrix3d rotate_z(double radians);
Eigen::Matrix3d rotation_matrix_from_axis_angle(const Eigen::Vector3d &axis, double radians);
Eigen::Matrix3d rotation_matrix_from_euler_zyx(const Eigen::Vector3d &e);
  Eigen::Matrix4d transformation_matrix(const Eigen::Matrix3d& r, const Eigen::Vector3d& p);
  //assigment 2------------------------------
Eigen::Vector3d euler_zyx_from_rotation(const Eigen::Matrix3d &r);
  Eigen::VectorXd screw_axis(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h);
  Eigen::MatrixXd adjoint_matrix(const Eigen::Matrix4d &tf);
  double cot(double x);
  Eigen::Matrix3d rotation_matrix_from_euler_yzx(const Eigen::Vector3d& e);
  Eigen::VectorXd wrench_sum();
  Eigen::Matrix3d matrix_exponential(const Eigen::Vector3d &w, double theta);
  std::pair<Eigen::Vector3d, double> matrix_logarithm(const Eigen::Matrix3d &r);
  Eigen::Matrix4d matrix_exponential(const Eigen::Vector3d &w, const Eigen::Vector3d &v, double theta);
  std::pair<Eigen::VectorXd, double> matrix_logarithm(const Eigen::Matrix4d &t);
  Eigen::Matrix3d G(const Eigen::Vector3d &w, const double &theta);
  Eigen::Matrix3d G_inv(const Eigen::Vector3d &w, const double &theta);
  std::pair<Eigen::VectorXd, double> matrix_logarithm(const Eigen::Matrix4d &t);
  //task 4
  void print_pose(const Eigen::Matrix4d &tf, const std::string &label = std::string());
  Eigen::Matrix4d planar_3r_fk_transform(const std::vector<double> &joint_positions);
  Eigen::Matrix4d planar_3r_fk_screw(const std::vector<double> &joint_positions);
  Eigen::Matrix4d ur3e_fk_screw(const std::vector<double> &joint_positions);
  Eigen::Matrix4d ur3e_fk_transform(const std::vector<double> &joint_positions);

  //assigemnt 3 -------------------------------------
  //task 1
  Eigen::VectorXd std_vector_to_eigen(const std::vector<double> &v);
  bool is_average_below_eps(const std::vector<double> &values, double eps, uint8_t n_values);
  std::pair<Eigen::Matrix4d, std::vector<Eigen::VectorXd>> ur3e_space_chain();
  Eigen::Matrix4d ur3e_space_fk(const Eigen::VectorXd &joint_positions);
  Eigen::Matrix4d matrix_exponential(const Eigen::VectorXd &screw, double theta); //redefining or somthing the matrix_exponentiale from assigemnt 1 or 3 to make work with task 4d
  std::pair<Eigen::Matrix4d, std::vector<Eigen::VectorXd>> ur3e_body_chain();
  Eigen::Matrix4d ur3e_body_fk(const Eigen::VectorXd &joint_positions);
  //task2
  std::pair<uint32_t, double> newton_raphson_root_find(const std::function<double(double)> &f, double x_0, double dx_0 = 0.5, double eps = 10e-7);
  std::pair<uint32_t, double> gradient_descent_root_find(const std::function<double(double)> &f, double x_0, double gamma = 0.01, double dx_0 = 0.5, double eps = 10e-7);
  //task 3
  Eigen::MatrixXd ur3e_space_jacobian(const Eigen::VectorXd &current_joint_positions);
  Eigen::MatrixXd ur3e_body_jacobian(const Eigen::VectorXd &current_joint_positions);
  std::pair<size_t, Eigen::VectorXd> ur3e_ik_body(const Eigen::Matrix4d &t_sd, const Eigen::VectorXd &current_joint_positions, double gamma = 1e-2, double v_e = 4e-3, double w_e = 4e-3);





}

#endif //MATH_H
