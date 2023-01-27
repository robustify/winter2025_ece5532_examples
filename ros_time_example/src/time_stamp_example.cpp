#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

ros::Time start_time;
ros::Publisher pub_twist_stamped;

void recvTwist(const geometry_msgs::TwistConstPtr& msg)
{
  geometry_msgs::TwistStamped twist_stamped_msg;

  twist_stamped_msg.header.stamp = ros::Time::now();
  twist_stamped_msg.twist = *msg;

  pub_twist_stamped.publish(twist_stamped_msg);

  ros::Duration time_since_start = twist_stamped_msg.header.stamp - start_time;
  ROS_INFO_STREAM("Received twist message " << time_since_start.toSec() << " seconds from start");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "time_stamp_example");
  ros::NodeHandle node;
  
  pub_twist_stamped = node.advertise<geometry_msgs::TwistStamped>("twist_stamped", 1);
  ros::Subscriber sub_twist = node.subscribe("twist", 1, recvTwist);

  start_time = ros::Time::now();

  ros::spin();
}