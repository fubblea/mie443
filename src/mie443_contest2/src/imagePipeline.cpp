#include "contest2/imagePipeline.h"
#include "contest2/contest2.h"
#include "contest2/utils.h"
#include "ros/console.h"
#include <contest2/imagePipeline.h>
#include <ctime>
#include <tuple>

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
  Mat descriptors_image;
  cv::resize(image, image, cv::Size(300, 300));
  detector->detectAndCompute(image, noArray(), keypoints_image,
                             descriptors_image);

  return std::make_tuple(keypoints_image, descriptors_image);
}

cv::Mat extractROI(const cv::Mat &inputImg) {
  int height = inputImg.rows;
  ROS_INFO("Image height is: %i", height);
  int width = inputImg.cols;

  cv::Mat croppedImg;

  if (height > 400 && MANUAL_CROP) {
    ROS_INFO("Big enough to crop");
    int cropX = width / MANUAL_CROP_X;
    int cropY = width / MANUAL_CROP_Y;
    int cropWidth = width / 2;
    int cropHeight = height - cropY;

    cv::Rect roi(cropX, cropY, cropWidth, cropHeight);

    croppedImg = inputImg(roi).clone();
    ROS_INFO("Cropped to manually");
  } else {
    ROS_INFO("Too short to manual crop, needs only gauss");
    croppedImg = inputImg;
  }

  if (GAUSSIAN_CROP) {
    cv::Mat gray, blurred, thresh;

    cv::cvtColor(croppedImg, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, CROP_SIZE, 0);
    // cv::threshold(blurred, thresh, MIN_CROP_THRESH, 255, cv::THRESH_BINARY);
    cv::adaptiveThreshold(
        gray, thresh, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
        cv::THRESH_BINARY_INV, ADAPT_BLOCK,
        ADAPT_CONST); // adaptive thresholding to account for lighting

    cv::threshold(
        gray, thresh, 0, 255,
        cv::THRESH_BINARY +
            cv::THRESH_OTSU); // OTSU tresholding to smooth out image noise

    // cv::imshow("Thresholded Image", thresh);
    //  find contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(thresh, contours, hierarchy, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    cv::Mat output;
    if (!contours.empty()) {
      std::vector<cv::Point> bestContour;
      double maxArea = 0;

      for (const auto &contour : contours) {
        double area = cv::contourArea(contour);
        if (area > maxArea) {
          maxArea = area;
          bestContour = contour;
        }
      }
      if (!bestContour.empty()) {
        cv::Rect box = cv::boundingRect(bestContour);
        ROS_INFO("Cropped using Gaussian filter");
        return croppedImg(box).clone();
      }
    }
  }

  return croppedImg;
}

std::tuple<int, float> ImagePipeline::getTemplateID(Boxes &boxes,
                                                    bool showView) {
  int template_id = -1;
  float best_match_per = -1;

  if (!isValid) {
    ROS_INFO("image not valid");
    std::cout << "ERROR: INVALID IMAGE!" << std::endl;
  } else if (img.empty() || img.rows <= 0 || img.cols <= 0) {
    std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
    std::cout << "img.empty():" << img.empty() << std::endl;
    std::cout << "img.rows:" << img.rows << std::endl;
    std::cout << "img.cols:" << img.cols << std::endl;
  } else {

    ROS_INFO("Cropping Image...");
    img = extractROI(img);

    // Find keypoints in scene (img) and compare to keypoint in templates
    std::vector<cv::KeyPoint> scannedKeypoints;
    cv::Mat scannedDescriptors;
    ROS_INFO("Getting image...");
    std::tie(scannedKeypoints, scannedDescriptors) =
        ImagePipeline::getFeatures(img); // feature extraction on scanned image

    // initialize image match parameters
    bool match_found;
    if (scannedDescriptors.rows > MIN_ROWS_BLANK) {
      std::tie(template_id, best_match_per, match_found) =
          ImagePipeline::imageMatch(scannedKeypoints, scannedDescriptors,
                                    showView);
    }
    if (best_match_per < 15) {
      template_id = -1;
    }
    ROS_INFO("Image match results: id: %i, best_match_per: %f, match_found: %i",
             template_id, best_match_per, match_found);

    /*if (showView) {
      cv::imshow("view", img);
    }
    cv::waitKey(10);*/
  }
  return std::make_tuple(template_id, best_match_per);
}

std::tuple<int, double, bool>
ImagePipeline::imageMatch(std::vector<cv::KeyPoint> &image_keypoints,
                          cv::Mat &image_descriptors, bool showView) {

  int matched_id = -1;
  float best_match_percentage = 0.0;
  ROS_INFO("%i descriptor rows in scanned image", image_descriptors.rows);

  if (image_descriptors.empty()) {
    ROS_INFO("No descriptors in scanned image");
    return std::make_tuple(matched_id, 0.0, matched_id != -1);
  }
  cv::FlannBasedMatcher flann(cv::makePtr<cv::flann::KDTreeIndexParams>(5));

  ROS_INFO("Memorized templates size: %zu", this->memorizedTemplates.size());

  for (size_t i = 0; i < this->memorizedTemplates.size(); i++) {
    ROS_INFO("%i descriptor rows in template image: %s",
             this->memorizedTemplates[i].template_descriptors.rows,
             getFileName(i).c_str());
    if (this->memorizedTemplates[i].template_descriptors.empty()) {
      ROS_WARN("Template descriptors are empty, skipping...");
      continue;
    }

    std::vector<cv::DMatch> matches;
    flann.match(this->memorizedTemplates[i].template_descriptors,
                image_descriptors, matches);

    std::sort(matches.begin(),
              matches.end()); // sort matches from best match to worst match

    double good_matches = 0;
    for (const auto &m : matches) {
      if (m.distance <
          MATCH_COMPARE_THRESH * matches.back().distance) { // lowe's ratio test
        good_matches++;
      }
    }
    ROS_INFO("Template %s, %f good matches found", getFileName(i).c_str(),
             good_matches);

    double percentMatch =
        (good_matches / this->memorizedTemplates[i].template_descriptors.rows) *
        100;
    ROS_INFO("Template %s, %f percent matched", getFileName(i).c_str(),
             percentMatch);
    if (percentMatch > best_match_percentage) {
      best_match_percentage = percentMatch;
      matched_id = i;
    }
  }
  cv::Mat img_matches;
  cv::drawMatches(cv::imread(TEMPLATE_FILES[matched_id],
                             cv::IMREAD_GRAYSCALE), // template image
                  this->memorizedTemplates[matched_id]
                      .template_keypoints, // template keypoints
                  img,                     // scanned image
                  image_keypoints,         // Scanned image keypoints
                  std::vector<cv::DMatch>(), img_matches, cv::Scalar::all(-1),
                  cv::Scalar::all(-1), std::vector<char>(),
                  cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  // Show the match visualization
  if (showView) {
    cv::imshow("Matches", img_matches);
  }
  cv::waitKey(0);
  if (best_match_percentage > MIN_CONF_THRESH) {
    return std::make_tuple(matched_id, best_match_percentage, matched_id != -1);
  } else {
    ROS_INFO("Did not meet MIN_CONF_THRESH: %f vs %f", best_match_percentage,
             MIN_CONF_THRESH);
    return std::make_tuple(-1, best_match_percentage, matched_id != -1);
  }
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
