#pragma once

// EXTERNAL HEADER FILES
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <std_msgs/String.h>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <iostream>
#include <vector>

// INTERNAL HEADER FILES
#include "contest2/boxes.h"

using namespace cv;
using std::cout;
using std::endl;

class MemorizedTemplate {
public:
  int template_name;
  std::vector<cv::KeyPoint> template_keypoints;
  cv::Mat template_descriptors;

public:
  MemorizedTemplate(int template_name,
                    std::vector<cv::KeyPoint> template_keypoints,
                    cv::Mat template_descriptors)
      : template_name(template_name), template_keypoints(template_keypoints),
        template_descriptors(template_descriptors) {};
};

class ImagePipeline {
private:
  cv::Mat img;
  bool isValid;
  image_transport::Subscriber sub;
  std::vector<MemorizedTemplate> memorizedTemplates;

public:
  ImagePipeline(ros::NodeHandle &n);
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  std::tuple<std::vector<cv::KeyPoint>, Mat> getFeatures(cv::Mat image);
  std::tuple<int, float> getTemplateID(Boxes &boxes, bool showView);
  std::tuple<int, double, bool>
  imageMatch(std::vector<cv::KeyPoint> &image_keypoints,
             cv::Mat &image_descriptors);
  void memorizeTemplates();
};
