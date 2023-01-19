#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

ros::Publisher pub_left;
ros::Publisher pub_right;

void recvTwist(const geometry_msgs::TwistConstPtr& msg)
{
  std_msgs::Float64 left_wheel_command;
  std_msgs::Float64 right_wheel_command;

  double rw = 0.3;
  double W = 1.2;
  double v = msg->linear.x;
  double pdot = msg->angular.z;

  left_wheel_command.data = (v - W * pdot / 2) / rw;
  right_wheel_command.data = (v + W * pdot / 2) / rw;

  pub_left.publish(left_wheel_command);
  pub_right.publish(right_wheel_command);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "diff_drive_node");
  ros::NodeHandle n;

  ros::Subscriber sub_twist = n.subscribe("cmd_vel", 1, recvTwist);
  pub_left = n.advertise<std_msgs::Float64>("left_speed", 1);
  pub_right = n.advertise<std_msgs::Float64>("right_speed", 1);

  ros::spin();
}
