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

// INTERNAL HEADER FILES
#include "contest2/boxes.h"

using namespace cv;
using std::cout;
using std::endl;

class ImagePipeline {
private:
  cv::Mat img;
  bool isValid;
  image_transport::Subscriber sub;

public:
  ImagePipeline(ros::NodeHandle &n);
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  std::tuple<std::vector<cv::KeyPoint>, Mat> getFeatures(cv::Mat image);
  int getTemplateID(Boxes &boxes, bool showView,
                    std::vector<std::string> template_names,
                    std::vector<std::vector<cv::KeyPoint>> template_keypoints,
                    std::vector<cv::Mat> template_descriptors);
  std::tuple<int, double, bool>
  imageMatch(const std::vector<std::string> &template_names,
             const std::vector<std::vector<cv::KeyPoint>> &template_keypoints,
             const std::vector<cv::Mat> &template_descriptors,
             const std::vector<cv::KeyPoint> &image_keypoints,
             const cv::Mat &image_descriptors, double &best_match_percentage);
  std::tuple<std::vector<std::string>, std::vector<std::vector<cv::KeyPoint>>,
             std::vector<cv::Mat>, bool>
  memorizeTemplates(std::vector<std::string> template_files,
                    std::vector<std::string> template_names,
                    std::vector<std::vector<cv::KeyPoint>> template_keypoints,
                    std::vector<cv::Mat> template_descriptors);
};
