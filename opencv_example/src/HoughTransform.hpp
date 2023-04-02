#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <opencv_example/HoughTransformConfig.h>

namespace opencv_example {

  class HoughTransform {
    public:
      HoughTransform(ros::NodeHandle n, ros::NodeHandle pn);

    private:
      void reconfig(opencv_example::HoughTransformConfig& config, uint32_t level);
      void recvImage(const sensor_msgs::ImageConstPtr& msg);

      ros::Subscriber sub_image_;

      dynamic_reconfigure::Server<HoughTransformConfig> srv_;
      HoughTransformConfig cfg_;

  };

}
