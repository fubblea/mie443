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
  Ptr<cv::xfeatures2d::SURF> detector =
      cv::xfeatures2d::SURF::create(MIN_HESSIAN);
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

cv::Mat extractROI(const cv::Mat &inputImg) {
  cv::Mat gray, blurred, thresh;
  cv::cvtColor(inputImg, gray, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
  cv::threshold(blurred, thresh, 200, 255, cv::THRESH_BINARY);

  // find contours
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(thresh, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  cv::Mat output;
  if (!contours.empty()) {
    double maxArea = 0;
    std::vector<cv::Point> bestContour;

    // find largest contour
    for (const auto &contour : contours) {
      double area = cv::contourArea(contour);
      if (area > maxArea) {
        maxArea = area;
        bestContour = contour;
      }
    }

    if (!bestContour.empty()) {
      std::vector<cv::Point> approx;
      cv::approxPolyDP(bestContour, approx,
                       0.02 * cv::arcLength(bestContour, true), true);

      if (approx.size() == 4) {
        std::sort(approx.begin(), approx.end(),
                  [](const cv::Point &a, const cv::Point &b) {
                    return a.x + a.y < b.x + b.y;
                  });

        cv::Point2f srcPoints[4] = {approx[2], approx[3], approx[1], approx[0]};

        float width = 2480, height = 3508;
        cv::Point2f dstPoints[4] = {
            {0, 0}, {width, 0}, {width, height}, {0, height}};

        cv::Mat matrix = cv::getPerspectiveTransform(srcPoints, dstPoints);
        cv::warpPerspective(inputImg, output, matrix, cv::Size(width, height));
      }
    }
  }

  return output;
}

int ImagePipeline::getTemplateID(Boxes &boxes, bool showView) {
  int template_id = -1;
  /*ROS_INFO("Cropping Image...");
  img = extractROI(img);*/
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
    if (scannedDescriptors.rows > MIN_ROWS_BLANK) {
      std::tie(template_id, best_match_per, match_found) =
          ImagePipeline::imageMatch(scannedKeypoints, scannedDescriptors,
                                    best_match_per);
    }
    ROS_INFO("Image match results: id: %i, best_match_per: %f, match_found: %i",
             template_id, best_match_per, match_found);

    if (showView) {
      cv::imshow("view", img);
    }
    cv::waitKey(10);
  }
  return template_id;
}

std::tuple<int, double, bool>
ImagePipeline::imageMatch(std::vector<cv::KeyPoint> &image_keypoints,
                          cv::Mat &image_descriptors,
                          double &best_match_percentage) {

  int matched_id = -1;
  best_match_percentage = 0.0;
  ROS_INFO("%i descriptor rows in scanned image", image_descriptors.rows);

  if (image_descriptors.empty()) {
    ROS_INFO("No descriptors in scanned image");
    return std::make_tuple(matched_id, 0.0, matched_id != -1);
  }
  cv::FlannBasedMatcher flann(cv::makePtr<cv::flann::KDTreeIndexParams>(5));

  ROS_INFO("Memorized templates size: %zu", this->memorizedTemplates.size());

  for (size_t i = 0; i < this->memorizedTemplates.size(); i++) {
    ROS_INFO("%i descriptor rows in template image",
             this->memorizedTemplates[i].template_descriptors.rows);
    if (this->memorizedTemplates[i].template_descriptors.empty()) {
      ROS_WARN("Template descriptors are empty, skipping...");
      continue;
    }

    std::vector<cv::DMatch> matches;
    flann.match(this->memorizedTemplates[i].template_descriptors,
                image_descriptors, matches);

    std::sort(matches.begin(), matches.end(),
              [](const cv::DMatch &a, const cv::DMatch &b) {
                return a.distance < b.distance;
              }); // sort matches from best match to worst match

    double good_matches = 0;
    for (const auto &m : matches) {
      if (m.distance <
          MATCH_COMPARE_THRESH * matches.back().distance) { // lowe's ratio test
        good_matches++;
      }
    }
    ROS_INFO("%f good matches found", good_matches);

    double percentMatch =
        (good_matches / this->memorizedTemplates[i].template_descriptors.rows) *
        100;
    ROS_INFO("%f percent matched", percentMatch);
    if (percentMatch > best_match_percentage) {
      best_match_percentage = percentMatch;
      matched_id = i;
    }
  }
  return std::make_tuple(matched_id, best_match_percentage, matched_id != -1);
}

void ImagePipeline::memorizeTemplates() {

  for (int i = 0; i < TEMPLATE_FILES.size(); i++) {
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

    MemorizedTemplate newMemorization =
        MemorizedTemplate(i, localKeypoints, localDescriptors);

    this->memorizedTemplates.push_back(newMemorization);
    ROS_INFO("Memorized this one. On to the next");
  }
}
