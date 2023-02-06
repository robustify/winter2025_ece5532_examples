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
  double n_val = N(lat);
  output.x() = (n_val + alt) * cos(lat) * cos(lon);
  output.y() = (n_val + alt) * cos(lat) * sin(lon);
  output.z() = (n_val * (1 - E2) + alt) * sin(lat);
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
  ref_ecef = computeEcef(ref_lat, ref_lon, ref_alt);
  current_ecef = computeEcef(current_lat, current_lon, current_alt);

  std::cout << std::fixed << "Reference ECEF coordinates:\n" << ref_ecef << "\n\n";
  std::cout << std::fixed << "Current ECEF coordinates:\n" << current_ecef << "\n\n";

  Eigen::Matrix3d enu_rot_mat;
  enu_rot_mat << -sin(ref_lon),                 cos(ref_lon),                0,
                 -sin(ref_lat) * cos(ref_lon), -sin(ref_lat) * sin(ref_lon), cos(ref_lat),
                  cos(ref_lat) * cos(ref_lon),  cos(ref_lat) * sin(ref_lon), sin(ref_lat);

  Eigen::Vector3d current_enu;
  current_enu = enu_rot_mat * (current_ecef - ref_ecef);

  std::cout << "Current ENU coordinates:\n" << current_enu << "\n\n";

  Eigen::Vector3d ref_utm;
  Eigen::Vector3d current_utm;
  std::string utm_zone;
  gps_common::LLtoUTM(ref_lat * 180.0 / M_PI, ref_lon * 180.0 / M_PI, ref_utm.y(), ref_utm.x(), utm_zone);
  gps_common::LLtoUTM(current_lat * 180.0 / M_PI, current_lon * 180.0 / M_PI, current_utm.y(), current_utm.x(), utm_zone);
  ref_utm.z() = ref_alt;
  current_utm.z() = current_alt;

  std::cout << "UTM zone: " << utm_zone << "\n";
  std::cout << std::fixed << "Reference UTM coordinates:\n" << ref_utm << "\n\n";
  std::cout << std::fixed << "Current UTM coordinates:\n" << current_utm << "\n\n";

  Eigen::Vector3d relative_utm;
  relative_utm = current_utm - ref_utm;
  std::cout << std::fixed << "Relative UTM position:\n" << relative_utm << "\n\n";

  // Calculate distance between current and reference point using both ENU and UTM
  std::cout << "ENU distance: " << current_enu.norm() << "  UTM distance: " << relative_utm.norm() << "\n";

  // Compare empirically-measured convergence angle with the theoretical convergence angle
  double angle = acos(current_enu.dot(relative_utm) / current_enu.norm() / relative_utm.norm());
  std::cout << "Empirical convergence angle: " << angle * 180.0 / M_PI << "\n";

  double zone_17_lon = -81.0 * M_PI / 180.0;
  double conv_angle = atan(tan(current_lon - zone_17_lon) * sin(current_lat));
  std::cout << "Theoretical convergence angle: " << conv_angle * 180.0 / M_PI << "\n";

}
