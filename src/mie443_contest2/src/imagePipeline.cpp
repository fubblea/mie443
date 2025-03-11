#include "contest2/imagePipeline.h"
#include "contest2/contest2.h"
#include "ros/console.h"
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
  Ptr<cv::xfeatures2d::SURF> detector =
      cv::xfeatures2d::SURF::create(minHessian);
  ROS_INFO("Detecting features");
  std::vector<KeyPoint> keypoints_image;
  ROS_INFO("Initializing keypoints");
  Mat descriptors_image;
  ROS_INFO("Initializing descriptors");
  detector->detectAndCompute(image, noArray(), keypoints_image,
                             descriptors_image);
  ROS_INFO("Detected keypoints and descriptors");

  return std::make_tuple(keypoints_image, descriptors_image);
}

int ImagePipeline::getTemplateID(
    Boxes &boxes, bool showView, std::vector<int> *template_names,
    std::vector<std::vector<cv::KeyPoint>> template_keypoints,
    std::vector<cv::Mat> template_descriptors) {
  int template_id = -1;
  if (!isValid) {
    ROS_INFO("image not valid");
    std::cout << "ERROR: INVALID IMAGE!" << std::endl;
  } else if (img.empty() || img.rows <= 0 || img.cols <= 0) {
    std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
    std::cout << "img.empty():" << img.empty() << std::endl;
    std::cout << "img.rows:" << img.rows << std::endl;
    std::cout << "img.cols:" << img.cols << std::endl;
  } else {

    // Find keypoints in scene (img) and compare to keypoint in templates
    std::vector<cv::KeyPoint> scannedKeypoints;
    cv::Mat scannedDescriptors;
    ROS_INFO("Getting image...");
    std::tie(scannedKeypoints, scannedDescriptors) =
        ImagePipeline::getFeatures(img); // feature extraction on scanned image

    // initialize image match parameters
    double best_match_per = 0.0;
    bool match_found;

    std::tie(template_id, best_match_per, match_found) =
        ImagePipeline::imageMatch(template_names, template_keypoints,
                                  template_descriptors, scannedKeypoints,
                                  scannedDescriptors, best_match_per);

    ROS_INFO("Image match results: id: %i, best_match_per: %f, match_found: %i",
             template_id, best_match_per, match_found);

    if (showView) {
      cv::imshow("view", img);
    }
    cv::waitKey(10);
  }
  return template_id;
}

std::tuple<int, double, bool> ImagePipeline::imageMatch(
    std::vector<int> *template_names,
    std::vector<std::vector<cv::KeyPoint>> &template_keypoints,
    std::vector<cv::Mat> &template_descriptors,
    std::vector<cv::KeyPoint> &image_keypoints, cv::Mat &image_descriptors,
    double &best_match_percentage) {

  int matched_id = -1;
  best_match_percentage = 0.0;

  if (image_descriptors.empty()) {
    ROS_INFO("No descriptors in scanned image");
    return std::make_tuple(matched_id, 0.0, matched_id != -1);
  }
  cv::FlannBasedMatcher flann(cv::makePtr<cv::flann::KDTreeIndexParams>(5));

  ROS_INFO("Template names size: %zu", template_names->size());

  for (size_t i = 0; i < template_names->size(); i++) {
    if (template_descriptors[i].empty()) {
      ROS_WARN("Template descriptors are empty, skipping...");
      continue;
    }

    std::vector<cv::DMatch> matches;
    flann.match(template_descriptors[i], image_descriptors, matches);

    double good_matches = 0;
    for (const auto &m : matches) {
      if (m.distance < 0.3 * matches.back().distance) { // lowe's ratio test
        good_matches++;
      }
    }

    double percentMatch = (good_matches / template_descriptors[i].rows) * 100;

    if (percentMatch > best_match_percentage) {
      best_match_percentage = percentMatch;
      matched_id = i;
    }
  }
  return std::make_tuple(matched_id, best_match_percentage, matched_id != -1);
}

std::tuple<std::vector<std::vector<cv::KeyPoint>>, std::vector<cv::Mat>, bool>
ImagePipeline::memorizeTemplates(
    std::vector<int> *template_names,
    std::vector<std::vector<cv::KeyPoint>> template_keypoints,
    std::vector<cv::Mat> template_descriptors) {

  for (int i = 0; i < 3; i++) {
    ROS_INFO("Reading template image");
    cv::Mat template_pic =
        cv::imread(TEMPLATE_FILES[i],
                   cv::IMREAD_GRAYSCALE); // read template image in grayscale
    ROS_INFO("Template path: %s", TEMPLATE_FILES[i].c_str());
    if (template_pic.empty()) {
      ROS_WARN("You done goofed. Check file path");
    }

    std::vector<cv::KeyPoint> localKeypoints;
    cv::Mat localDescriptors;
    ROS_INFO("Starting feature detection");
    std::tie(localKeypoints, localDescriptors) =
        ImagePipeline::getFeatures(template_pic);
    ROS_INFO("Feature detection completed");
    template_names->push_back(i);
    template_keypoints.push_back(localKeypoints);
    template_descriptors.push_back(localDescriptors);
    ROS_INFO("Memorized this one. On to the next");
  }
  return std::make_tuple(template_keypoints, template_descriptors, true);
}
