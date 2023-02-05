#include <eigen3/Eigen/Dense>
#include <iostream>  // For cout
#include <gps_common/conversions.h> // UTM conversion functions

#define A 6378137.0
#define E2 6.6943799014e-3
double N(double lat) {
  return A / sqrt(1.0 - E2 * sin(lat) * sin(lat));
}

Eigen::Vector3d computeEcef(double lat, double lon, double alt) {
  Eigen::Vector3d output;
  // TODO: Implement ECEF conversion calculations here
  return output;
}

int main(int argc, char** argv)
{
  double ref_lat = 42.6713972 * M_PI / 180.0;
  double ref_lon = -83.2156750 * M_PI / 180.0;
  double ref_alt = 281.0;

  double current_lat = 42.6708444 * M_PI / 180.0;
  double current_lon = -83.2141333 * M_PI / 180.0;
  double current_alt = 278.0;

  Eigen::Vector3d ref_ecef;
  Eigen::Vector3d current_ecef;
  // TODO: Populate ECEF vectors by calling computeEcef

  std::cout << std::fixed << "Reference ECEF coordinates:\n" << ref_ecef << "\n\n";
  std::cout << std::fixed << "Current ECEF coordinates:\n" << current_ecef << "\n\n";

  Eigen::Matrix3d enu_rot_mat;
  // TODO: Fill in ENU rotation matrix using reference geodetic coordinates

  Eigen::Vector3d current_enu;
  // TODO: Calculate current ENU coordinates using above calculated values

  std::cout << "Current ENU coordinates:\n" << current_enu << "\n\n";

  Eigen::Vector3d ref_utm;
  Eigen::Vector3d current_utm;
  std::string utm_zone;
  // TODO: Compute UTM coordinates of reference point and current position

  std::cout << "UTM zone: " << utm_zone << "\n";
  std::cout << std::fixed << "Reference UTM coordinates:\n" << ref_utm << "\n\n";
  std::cout << std::fixed << "Current UTM coordinates:\n" << current_utm << "\n\n";

  Eigen::Vector3d relative_utm;
  // TODO: Calculate relative position of the current position from the reference point in UTM
  std::cout << std::fixed << "Relative UTM position:\n" << relative_utm << "\n\n";

  // Calculate distance between current and reference point using both ENU and UTM
  std::cout << "ENU distance: " << current_enu.norm() << "  UTM distance: " << relative_utm.norm() << "\n";

  // Compare empirically-measured convergence angle with the theoretical convergence angle
  double angle = acos(current_enu.dot(relative_utm) / current_enu.norm() / relative_utm.norm());
  std::cout << "Empirical convergence angle: " << angle * 180.0 / M_PI << "\n";

  double zone_17_lon = -81.0 * M_PI / 180.0;
  double conv_angle = atan(tan(current_lon - zone_17_lon) * sin(current_lat));
  std::cout << "Theoretical convergence angle: " << conv_angle << "\n";//* 180.0 / M_PI << "\n";

}
