#include <roundbot_local_planner/RoundbotLocalPlanner.hpp>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(roundbot_local_planner::RoundbotLocalPlanner, nav_core::BaseLocalPlanner)

namespace roundbot_local_planner{

RoundbotLocalPlanner::RoundbotLocalPlanner() :
  reached_goal_(false),
  initialized_(false)
{
}

RoundbotLocalPlanner::RoundbotLocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) :
  reached_goal_(false),
  initialized_(false)
{
  initialize(name, tf, costmap_ros);
}

// This method is called by move_base whenever it is ready for a new speed and yaw rate command
// If a 'false' is returned, it tells move_base to trigger a new global plan operation
// If a 'true' is returned, no new global plan will be generated
// The 'cmd_vel' argument is a reference to a Twist message.  Modify this with the speed and yaw rate
//    commands to send the output.
bool RoundbotLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  boost::lock_guard<boost::mutex> guard(mtx_);

  // Abort and stop the robot if global plan is empty
  if (plan_.size() == 0){
    ROS_ERROR("Empty plan... Setting velocity to zero!");
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return false;
  }

  // Lookup base_footprint pose in global frame
  geometry_msgs::TransformStamped base_pose;
  tf2::Transform base_pose_tf;
  try {
    base_pose = listener_->lookupTransform(plan_frame_, base_frame_, ros::Time(0));
    tf2::convert(base_pose.transform, base_pose_tf);
  } catch (...) {
    ROS_ERROR("TF lookup failed!");
    return false;
  }

  // Extract just the position of the base from the complete transform
  tf2::Vector3 base_position = base_pose_tf.getOrigin();

  // Search the global plan path for the index of the closest point
  int closest_idx = findPathPoint(base_position);

  // Distance between global plan points is half of the costmap resolution
  double plan_resolution = 0.5 * costmap_->getCostmap()->getResolution();

  /****** Calculate speed command ******/
  // Calculate a plan point array index to roughly match the specified distance in front of the vehicle
  // This index is the lookahead target point for speed and steering control.
  int lookahead_idx = std::min(closest_idx + (int)(lookahead_distance / plan_resolution), (int)plan_.size() - 1);

  // Compute relative position from vehicle to lookahead point, and compute heading error in a similar fasion
  // to the recommened approach for the GPS simulation project
  double dx = plan_[lookahead_idx].pose.position.x - base_position.x();
  double dy = plan_[lookahead_idx].pose.position.y - base_position.y();
  double goal_angle = atan2(dy, dx);
  double heading_angle, pitch, roll;
  tf2::Matrix3x3(base_pose_tf.getRotation()).getEulerYPR(heading_angle, pitch, roll);
  double heading_error = angles::shortest_angular_distance(heading_angle, goal_angle);

  // Multiplicative scale factor for speed based on heading error.
  //  - If heading error is greater than 45 degrees, stop completely
  //  - If heading error is less than 30 degrees, go full speed
  //  - Linear interpolation between 30 and 45 degrees of error
  double speed_scale_factor = 1.0;
  if (std::abs(heading_error) > M_PI/3) {
    speed_scale_factor = 0.0;
  } else if (std::abs(heading_error) > M_PI/6) {
    speed_scale_factor = 1.0 - (std::abs(heading_error) - M_PI/6) / (M_PI/3 - M_PI/6);
  }
  cmd_vel.linear.x = target_speed * speed_scale_factor;
  /**********/

  /****** Calculate yaw rate command ******/
  // Project target point from global frame into vehicle local frame
  tf2::Vector3 tf_plan_point(plan_[lookahead_idx].pose.position.x, plan_[lookahead_idx].pose.position.y, 0);
  tf_plan_point = base_pose_tf.inverse() * tf_plan_point;

  // Compute a turning radius such that the vehicle hits the target point
  double radius;
  
  if (std::abs(tf_plan_point.y()) > 1e-3) {
    radius = 0.5 * (tf_plan_point.x() * tf_plan_point.x() + tf_plan_point.y() * tf_plan_point.y()) / tf_plan_point.y();
  } else {
    radius = 1e8;
  }

  // Use the target speed and computed radius to compute a yaw rate command
  cmd_vel.angular.z = target_speed / radius;
  /**********/

  /****** Detect imminent collision and stop ******/
  // Project the plan index forward 1 meter to check if a collision is imminent
  int collision_check_idx = std::min(closest_idx + (int)(1.0 / plan_resolution), (int)plan_.size() - 1);
  if (!checkCost(closest_idx, collision_check_idx)) {
    cmd_vel.linear.x = 0;
  }
  /**********/

  /****** Detect upcoming obstruction and request a new plan from global planner ******/
  // Calculate a plan point array index to roughly match the replan distance
  int replan_idx = std::min(closest_idx + (int)(replan_distance / plan_resolution), (int)plan_.size() - 1);
  bool new_plan_required = !checkCost(closest_idx, replan_idx);
  /**********/

  // See if goal point (last one in the plan array) is reached within tolerance
  tf2::Vector3 endpoint(plan_.back().pose.position.x, plan_.back().pose.position.y, 0.0);
  tf2::Vector3 delta = endpoint - base_position;
  reached_goal_ = (delta.length2() < goal_dist_tolerance * goal_dist_tolerance);

  // Return false if a new plan is required to request a new one from the global planner.
  // Return true if the current plan is OK.
  return !new_plan_required;
}

// This method checks the costmap cells between two global plan points, and returns false if any of the costmap cells
// are inside the inscribed radius (cyan color in Rviz)
bool RoundbotLocalPlanner::checkCost(int start_idx, int end_idx)
{
  for (int i = start_idx; i <= end_idx; i++) {
    unsigned int mx, my;
    bool point_on_map = costmap_->getCostmap()->worldToMap(plan_[i].pose.position.x, plan_[i].pose.position.y, mx, my);
    if (point_on_map) {
      if (costmap_->getCostmap()->getCost(mx, my) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
        return false;
      }
    }
  }
  return true;
}

// This method is called by move_base once at startup
void RoundbotLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  // Set class properties with input arguments so they can be used in other class methods.
  listener_ = tf;
  costmap_ = costmap_ros;
  name_ = name;

  // Look up ROS parameters
  ros::NodeHandle pn("~/" + name);
  pn.param("lookahead_distance", lookahead_distance, 1.0);
  pn.param("target_speed", target_speed, 1.0);
  pn.param("replan_distance", replan_distance, 3.0);
  pn.param("goal_dist_tolerance", goal_dist_tolerance, 0.5);

  // Initialize base frame ID from costmap instance
  base_frame_ = costmap_->getBaseFrameID();

  initialized_ = true;
}

// This method is called by move_base after a new global plan becomes available.  This new
// plan is passed as the 'plan' argument
bool RoundbotLocalPlanner::setPlan(const std::vector< geometry_msgs::PoseStamped >& plan)
{
  reached_goal_ = false;

  if (!initialized_){
    ROS_ERROR("Local planner not initialized!");
    return false;
  }

  // If plan is not empty, set the class property 'plan_' with the plan and also latch the particular
  // global frame ID in 'plan_frame_'.
  if (plan.size() > 0) {
    try {
      geometry_msgs::TransformStamped transform_msg;
      tf2::Transform transform_tf;
      tf2::convert(transform_msg.transform, transform_tf);
      transform_msg = listener_->lookupTransform(plan[0].header.frame_id, base_frame_, ros::Time(0));
    } catch (...) {
      ROS_ERROR("No transform found between [%s] and planned path frame [%s]", base_frame_.c_str(), plan[0].header.frame_id.c_str());
      return false;
    }
    boost::lock_guard<boost::mutex> guard(mtx_);
    plan_ = plan;
    plan_frame_ = plan[0].header.frame_id;
  }else{
    boost::lock_guard<boost::mutex> guard(mtx_);
    plan_.clear();
  }
  return true;
}

int RoundbotLocalPlanner::findPathPoint(const tf2::Vector3& base_position)
{
  int min_idx = -1;
  double min_dist = INFINITY;

  for (size_t i = 0; i < plan_.size(); i++){
    double l2 = (tf2::Vector3(plan_[i].pose.position.x, plan_[i].pose.position.y, 0.0) - base_position).length2();
    if (l2 < min_dist){
      min_dist = l2;
      min_idx = i;
    }
  }
  return min_idx;
}

bool RoundbotLocalPlanner::isGoalReached()
{
  return reached_goal_;
}

}
