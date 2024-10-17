#include <iostream>

#include <Eigen/Dense>
#include "math/math.h"








int main()
{
    Eigen::Vector3d e = Eigen::Vector3d{60.0, 45.0, 30.0} * math::deg_to_rad;
    Eigen::Matrix3d r = math::rotation_matrix_from_euler_zyx(e);
    Eigen::Vector3d ea = math::euler_zyx_from_rotation(r);
    std::cout << " E: " << e.transpose() * math::rad_to_deg << std::endl;
    std::cout << "Ea: " << ea.transpose() * math::rad_to_deg << std::endl;

    //task 2 a
    std::cout <<"`\n Task 2 " << std::endl;
    Eigen::Vector3d fw (-30,0,0) ;
    Eigen::Vector3d ts (0,0,2);
    Eigen::Vector3d ews = Eigen::Vector3d{60,-60, 0} * math::deg_to_rad;

    Eigen::Matrix3d R_ws = math::rotation_matrix_from_euler_yzx(ews);
    Eigen::Vector3d fs = R_ws.transpose() * fw;
    Eigen::Vector3d tw = R_ws * ts;


    std::cout <<  "force world: " << fw.transpose() << "\n" << std::endl;
    std::cout <<  "torque world : " << tw.transpose() << "\n" << std::endl;
    std::cout <<  "force sensor: " << fs.transpose() << "\n" << std::endl;
    std::cout <<  "torque sensor : " << ts.transpose() << "\n" << std::endl;

    Eigen::VectorXd ff = math::wrench_sum();
    std::cout << "example result 2b; \n ff " << ff.transpose() << std::endl;

    std::cout <<"\n --------------NEW TASK----------------------" << std::endl;

    //task 4
    std::cout <<"`\n Task 4b test" << std::endl;

    std::vector<std::vector<double>> joint_test_position = {
        {0.0, 0.0, 0.0},
        {90.0, 0.0, 0.0},
        {0.0, 90.0, 0.0},
        {0.0, 0.0, 90.0},
        {10.0, -15.0, 2.75}
    };
    /*for (size_t i = 0; i < joint_test_position.size(); ++i) {
        std::vector<double> joint_positions = joint_test_position[i];
        Eigen::Matrix4d pose = math::planar_3r_fk_transform(joint_positions);

        // Print the Euler ZYX angles and linear position
        math::print_pose("Configuration " +  std::to_string(i + 1), pose);
        std::cout << std::endl;*/
    std::cout << "\nForward kinematics using transformation matrices\n" << std::endl;
    for (const std::vector<double> &positions : joint_test_position) {
        Eigen::Matrix4d T = math::planar_3r_fk_transform(positions);
        std::cout << "----------------------------------------" << std::endl;
        math::print_pose(T, "Pose for joint positions: [" + std::to_string(positions[0]) + ", " + std::to_string(positions[1]) + ", " + std::to_string(positions[2]) + "]");
    }


    std::cout <<"\nTask 4c Test" << std::endl;

    std::cout << "\nCalculate forward kinematics using the product of exponentials\n" << std::endl;
    for (const std::vector<double> &positions : joint_test_position) {
        Eigen::Matrix4d Task4c = math::planar_3r_fk_screw(positions);
        std::cout << "----------------------------------------" << std::endl;
        math::print_pose(Task4c, "Pose for joint positions: [" + std::to_string(positions[0]) + ", " + std::to_string(positions[1]) + ", " + std::to_string(positions[2]) + "]");
    }

    std::cout <<"\n ----------------NEW TASK--------------------" << std::endl;


    //task 5a


    std::vector<std::vector<double>> joint_test_position_5 = {
        {0.0, 0.0, 0.0, -90.0, 0.0, 0.0},
        {0.0, -180, 0.0, 0.0, 0.0, 0.0},
        {0.0, -90, 0.0, 0.0, 0.0, 0.0}
    };

    std::cout <<"\nTask 5a " << std::endl;

    std::cout << "\nCalculate forward kinematics using the product of exponentials\n" << std::endl;
    for (const std::vector<double> &positions : joint_test_position_5)
    {
        Eigen::Matrix4d Task5a = math::ur3e_fk_screw(positions);
        std::cout << "----------------------------------------" << std::endl;
        math::print_pose(Task5a, "Task 5 pose for joint positions: [" + std::to_string(positions[0]) + ", " + std::to_string(positions[1]) + ", " + std::to_string(positions[2]) + ", " + std::to_string(positions[3]) + ", " + std::to_string(positions[4]) + ", " + std::to_string(positions[5]) + "]");

    }
    return 0;
}


