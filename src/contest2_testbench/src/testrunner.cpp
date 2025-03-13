#include "diagnostic_msgs/KeyValue.h"
#include "ros/console.h"
#include "ros/init.h"
#include <contest2/boxes.h>
#include <contest2/imagePipeline.h>
#include <contest2/utils.h>
#include <contest2_testbench/testrunner.h>
#include <string>

#define IMAGE_TOPIC "camera/rgb/image_raw"

std::string lastImageId;
bool updateSync = false;

void syncCallback(const sensor_msgs::ImageConstPtr &msg) {
  std::string currImageId = msg->header.frame_id;

  if (currImageId != lastImageId) {
    ROS_INFO("Got a new image: %s. Updating sync", currImageId.c_str());
    lastImageId = currImageId;
    updateSync = true;
  } else {
    ROS_INFO("Still the same image: %s", currImageId.c_str());
    updateSync = false;
  }
}

bool cmdOptionExists(char **begin, char **end, const std::string &option) {
  return std::find(begin, end, option) != end;
}

int main(int argc, char **argv) {
  // Parse args
  bool showView = true;
  if (cmdOptionExists(argv, argv + argc, "-hideView")) {
    showView = false;
  }

  ros::init(argc, argv, "testrunner_node");
  ros::NodeHandle nh;

  ImagePipeline imagePipeline(nh);
  imagePipeline.memorizeTemplates();

  // Set up publisher for acknowledgment on "image_ack" topic
  ros::Publisher ack_pub;
  ack_pub = nh.advertise<diagnostic_msgs::KeyValue>("image_ack", 1);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(IMAGE_TOPIC, 1, &syncCallback);

  // Initialize box coordinates and templates
  Boxes boxes;
  if (!boxes.load_coords() || !boxes.load_templates()) {
    std::cout << "ERROR: could not load coords or templates" << std::endl;
    return -1;
  }
  for (int i = 0; i < boxes.coords.size(); ++i) {
    std::cout << "Box coordinates: " << std::endl;
    std::cout << i << " x: " << boxes.coords[i][0]
              << " y: " << boxes.coords[i][1] << " z: " << boxes.coords[i][2]
              << std::endl;
  }

  ros::Rate loop_rate(10); // subscribing rate

  diagnostic_msgs::KeyValue ack;
  while (ros::ok()) {
    ros::spinOnce();

    if (updateSync) {
      std::tuple<int, float> guess =
          imagePipeline.getTemplateID(boxes, showView);
      ROS_INFO("Guess: %s", getFileName(std::get<0>(guess)).c_str());

      ack.key = getFileName(std::get<0>(guess)).c_str();
      ack.value = std::to_string(std::get<1>(guess));
      ack_pub.publish(ack);
    }

    loop_rate.sleep();
  }

  return 0;
}
ros::Publisher ack_pub;
