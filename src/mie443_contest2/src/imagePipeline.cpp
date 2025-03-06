#include "imagePipeline.h"
#include <contest2/imagePipeline.h>
#include <ctime>

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC                                                            \
  "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

ImagePipeline::ImagePipeline(ros::NodeHandle &n) {
  image_transport::ImageTransport it(n);
  sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
  isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  try {
    if (isValid) {
      img.release();
    }
    img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
    isValid = true;
  } catch (cv_bridge::Exception &e) {
    std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
              << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
    isValid = false;
  }
}

std::tuple<std::vector<cv::KeyPoint>, Mat>
ImagePipeline::getFeatures(cv::Mat image) {
  int minHessian = 400; // try changing it and see what it does?
  Ptr<SURF> detector = SURF::create(minHessian);
  std::vector<KeyPoint> keypoints_image;
  Mat descriptors_image;
  detector->detectAndCompute(image, noArray(), keypoints_image,
                             descriptors_image);

  return std::make_tuple(keypoints_image, descriptors_image);
}

int ImagePipeline::getTemplateID(Boxes &boxes, bool showView) {
  int template_id = -1;
  if (!isValid) {
    std::cout << "ERROR: INVALID IMAGE!" << std::endl;
  } else if (img.empty() || img.rows <= 0 || img.cols <= 0) {
    std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
    std::cout << "img.empty():" << img.empty() << std::endl;
    std::cout << "img.rows:" << img.rows << std::endl;
    std::cout << "img.cols:" << img.cols << std::endl;
  } else {

    // Find keypoints in scene (img) and compare to keypoint in templates
    sleep(3);

    if (showView) {
      cv::imshow("view", img);
    }
    cv::waitKey(10);
  }
  return template_id;
}

std::tuple<std::string, double, bool> ImagePipeline::ImageMatch(
    const std::vector<std::string> &template_names,
    const std::vector<std::vector<cv::KeyPoint>> &template_keypoints,
    const std::vector<cv::Mat> &template_descriptors,
    const std::vector<cv::KeyPoint> &image_keypoints,
    const cv::Mat &image_descriptors, double &best_match_percentage) {

  std::string matched_tag = "None";
  best_match_percentage = 0.0;

  if (image_descriptors.empty()) {
    ROS_INFO("No descriptors in scanned image");
    return std::make_tuple("None", 0.0, false);
  }
  cv::FlannBasedMatcher flann(cv::makePtr<cv::flann::KDTreeIndexParams>(5));

  for (size_t i = 0; i < template_names.size(); i++) {
    if (template_descriptors[i].empty())
      continue;

    std::vector<cv::DMatch> matches;
    flann.match(template_descriptors[i], image_descriptors, matches);

    double good_matches = 0;
    for (const auto &m : matches) {
      if (m.distance < 0.3 * matches.back().distance) {
        good_matches++;
      }
    }

    double percentMatch = (good_matches / template_descriptors[i].rows) * 100;

    if (percentMatch > best_match_percentage) {
      best_match_percentage = percentMatch;
      matched_tag = template_names[i];
    }
  }
  return std::make_tuple(matched_tag, best_match_percentage, true);
}
