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
    int minHessian = 400; // try changing it and see what it does?
    Ptr<SURF> detector = SURF::create(minHessian);
    std::vector<KeyPoint> keypoints_scene;
    Mat descriptors_scene;
    detector->detectAndCompute(img, noArray(), keypoints_scene,
                               descriptors_scene);

    Ptr<DescriptorMatcher> matcher =
        DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    std::vector<std::vector<DMatch>> knn_matches;

    // iterate
    matcher->knnMatch(templates, descriptors_scene, knn_matches,
                      2); // compare templates with scene.. iterate?

    const float ratio_thresh = 0.75f;

    if (showView) {
      cv::imshow("view", img);
    }
    cv::waitKey(10);
  }
  return template_id;
}
