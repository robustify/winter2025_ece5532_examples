#include <eigen3/Eigen/Dense>
#include <iostream>

int main(int argc, char** argv) {

  Eigen::Vector3d translation;
  translation << 14, 14, 0;

  double psi = 120 * M_PI / 180;
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix << cos(psi), -sin(psi), 0,
                     sin(psi), cos(psi), 0,
                     0, 0, 1;

  Eigen::Vector3d global_coordinates;
  global_coordinates << 5, 27, 0;

  Eigen::Vector3d vehicle_coordinates = rotation_matrix.transpose() * (global_coordinates - translation);

  std::cout << "Goal point in vehicle coordinates: " << std::endl << vehicle_coordinates << std::endl;

  return 0;
}
