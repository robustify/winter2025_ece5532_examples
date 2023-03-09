#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <nav_core/base_local_planner.h>
#include <angles/angles.h>
#include <boost/thread/shared_mutex.hpp>

namespace roundbot_local_planner{

  class RoundbotLocalPlanner : public nav_core::BaseLocalPlanner {
    public:
      RoundbotLocalPlanner();
      RoundbotLocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

      // Overridden methods of BaseLocalPlanner
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
      void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
      bool isGoalReached();
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

    private:
      int findPathPoint(const tf2::Vector3& base_position);
      bool checkCost(int start_idx, int end_idx);
      double radiusFromPoint(double x, double y);

      std::vector<geometry_msgs::PoseStamped> plan_;

      double lookahead_distance;
      double target_speed;
      double replan_distance;
      double goal_dist_tolerance;
      double stop_radius;
      double max_speed_radius;

      std::string name_;
      std::string plan_frame_;
      std::string base_frame_;
      tf2_ros::Buffer* listener_;
      costmap_2d::Costmap2DROS* costmap_;
      bool reached_goal_;
      bool initialized_;
      boost::mutex mtx_;
  };

}