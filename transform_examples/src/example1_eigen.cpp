#include <eigen3/Eigen/Dense>
#include <iostream>

int main(int argc, char** argv) {

  Eigen::Vector3d translation;
  translation << 10, 3, 0;

  double psi = 120 * M_PI / 180;
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix << cos(psi), -sin(psi), 0,
                     sin(psi), cos(psi), 0,
                     0, 0, 1;

  Eigen::Vector3d vehicle_coordinates;
  vehicle_coordinates << 25, 0, 0;

  Eigen::Vector3d global_coordinates = rotation_matrix * vehicle_coordinates + translation;

  std::cout << "Target position in global coordinates: " << std::endl << global_coordinates << std::endl;

  return 0;
}
