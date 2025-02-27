#pragma once

#include <boxes.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <std_msgs/String.h>

class ImagePipeline {
private:
  cv::Mat img;
  bool isValid;
  image_transport::Subscriber sub;

public:
  ImagePipeline(ros::NodeHandle &n);
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  int getTemplateID(Boxes &boxes, bool showView = true);
};
