#include <contest2_testbench/testbench.h>

namespace fs = boost::filesystem;

bool ack_received = false;

// Callback for acknowledgment messages
void ackCallback(const std_msgs::Bool::ConstPtr &msg) {
  if (msg->data) {
    ack_received = true;
    ROS_INFO("Received acknowledgment from subscriber.");
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "testbench_node");
  ros::NodeHandle nh;

  // Use image_transport for publishing images
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/rgb/image_raw", 1);

  // Subscribe to the ack topic
  ros::Subscriber ack_sub = nh.subscribe("image_ack", 1, ackCallback);

  // Get list of image files from the folder (assumes common image extensions)
  std::vector<std::string> image_files;
  if (fs::exists(IMAGE_DATABASE_PATH) &&
      fs::is_directory(IMAGE_DATABASE_PATH)) {
    for (auto &entry : fs::directory_iterator(IMAGE_DATABASE_PATH)) {
      std::string ext = fs::extension(entry.path());
      if (ext == ".jpg" || ext == ".png" || ext == ".bmp") {
        image_files.push_back(entry.path().string());
      }
    }
  } else {
    ROS_ERROR_STREAM("Folder " << IMAGE_DATABASE_PATH
                               << " does not exist or is not a directory.");
    return -1;
  }

  // Ensure we have images to publish
  if (image_files.empty()) {
    ROS_ERROR("No image files found in the specified folder.");
    return -1;
  }

  ros::Rate loop_rate(10); // publishing rate

  size_t idx = 0;
  int totalImages = image_files.size();
  while (ros::ok()) {
    // Load image using OpenCV
    cv::Mat img = cv::imread(image_files[idx], cv::IMREAD_COLOR);
    if (img.empty()) {
      ROS_WARN_STREAM("Could not load image: " << image_files[idx]);
    } else {
      // Convert the image to a ROS image message (using cv_bridge)
      std_msgs::Header header;
      header.stamp = ros::Time::now();
      header.frame_id = "camera_frame";
      sensor_msgs::ImageConstPtr msg =
          cv_bridge::CvImage(header, "bgr8", img).toImageMsg();

      // Publish the image message
      pub.publish(msg);
      ROS_INFO("Published image [%zu/%i]: %s", idx + 1, totalImages,
               image_files[idx].c_str());
    }

    if (ack_received) {
      // Move to next image, loop back at the end
      idx = (idx + 1) % totalImages;
      ack_received = false;
    } else {
      ROS_INFO("Waiting for acknowledgment from subscriber");
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
