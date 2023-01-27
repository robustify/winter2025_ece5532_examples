#include <ros/ros.h>

void timerCallback(const ros::TimerEvent& event)
{
  if (event.last_real == ros::Time(0)) {
    return;
  }

  ros::Duration actual_time_diff = event.current_real - event.last_real;
  ros::Duration expected_time_diff = event.current_expected - event.last_expected;

  ROS_INFO_STREAM("Actual time since last trigger: " << actual_time_diff.toSec());
  ROS_INFO_STREAM("Expected time difference: " << expected_time_diff.toSec() << "\n");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "timer_example");
  ros::NodeHandle node;

  ros::Timer timer = node.createTimer(ros::Duration(0.1), timerCallback);
  
  ros::spin();
}