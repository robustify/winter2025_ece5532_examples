#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv) {

  tf2::Vector3 translation(10, 3, 0);
  tf2::Quaternion q;
  q.setRPY(0, 0, 120 * M_PI / 180);

  tf2::Transform transform;
  transform.setOrigin(translation);
  transform.setRotation(q);

  tf2::Vector3 vehicle_coordinates(25, 0, 0);

  tf2::Vector3 global_coordinates = transform * vehicle_coordinates;

  printf("Target position in global coordinates: (%f, %f, %f)\n", global_coordinates.x(),
                                                                  global_coordinates.y(),
                                                                  global_coordinates.z());

  return 0;
}