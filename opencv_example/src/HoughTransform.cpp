#include "HoughTransform.hpp"

namespace opencv_example {

HoughTransform::HoughTransform(ros::NodeHandle n, ros::NodeHandle pn)
{
  sub_image_ = n.subscribe("raw_image", 1, &HoughTransform::recvImage, this);

  srv_.setCallback(boost::bind(&HoughTransform::reconfig, this, _1, _2));

  cv::namedWindow("Raw Image", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Blue Image", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Thres Image", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Erode Image", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Dilate Image", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Canny Image", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Lines Image", cv::WINDOW_AUTOSIZE);
}

void HoughTransform::recvImage(const sensor_msgs::ImageConstPtr& msg)
{
  // Convert raw image from ROS image message into a cv::Mat
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  cv::Mat raw_img = cv_ptr->image;

  cv::imshow("Raw Image", raw_img);
  cv::waitKey(1);

  // Split RGB image into its three separate channels
  std::vector<cv::Mat> split_images;
  cv::split(raw_img, split_images);

  // TODO: Extract the blue channel into its own grayscale image
  cv::Mat blue_image;

  cv::imshow("Blue Image", blue_image);
  cv::waitKey(1);

  // TODO: Apply binary threshold to create a binary image where white pixels correspond to high blue values
  cv::Mat thres_img;

  cv::imshow("Thres Image", thres_img);
  cv::waitKey(1);

  // TODO: Apply erosion to clean up noise
  cv::Mat erode_img;

  cv::imshow("Erode Image", erode_img);
  cv::waitKey(1);

  // TODO: Apply dilation to expand regions that passed the erosion filter
  cv::Mat dilate_img;

  cv::imshow("Dilate Image", dilate_img);
  cv::waitKey(1);

  // Apply Canny edge detection to reduce the number of points that are passed to Hough Transform
  cv::Mat canny_img;
  cv::Canny(dilate_img, canny_img, 1, 2);
  cv::imshow("Canny Image", canny_img);

  // TODO: Run Probabilistic Hough Transform algorithm to detect line segments
  std::vector<cv::Vec4i> line_segments;

  // Draw detected Hough lines onto the raw image for visualization
  for (int i=0; i<line_segments.size(); i++){
    cv::line(raw_img, cv::Point(line_segments[i][0], line_segments[i][1]),
      cv::Point(line_segments[i][2], line_segments[i][3]), cv::Scalar(0, 0, 255));
  }

  cv::imshow("Lines Image", raw_img);
  cv::waitKey(1);

}

void HoughTransform::reconfig(opencv_example::HoughTransformConfig& config, uint32_t level)
{

  // Force erosion and dilation filter sizes to be an odd number
  if ((config.erode_size % 2) == 0) {
    config.erode_size--;
  }

  if ((config.dilate_size % 2) == 0) {
    config.dilate_size--;
  }

  cfg_ = config;
}

}
