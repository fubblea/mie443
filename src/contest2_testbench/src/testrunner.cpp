#include "ros/console.h"
#include "ros/init.h"
#include <contest2/boxes.h>
#include <contest2/imagePipeline.h>
#include <contest2_testbench/testrunner.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "testrunner_node");
  ros::NodeHandle nh;

  ImagePipeline imagePipeline(nh);

  // Set up publisher for acknowledgment on "image_ack" topic
  ros::Publisher ack_pub;
  ack_pub = nh.advertise<std_msgs::Bool>("image_ack", 1);

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

  std_msgs::Bool ack;
  while (ros::ok()) {
    ros::spinOnce();

    std::tuple<int, float> guess = imagePipeline.getTemplateID(boxes, false);
    ROS_INFO("Guess: %i", std::get<0>(guess));

    ack.data = true;
    ack_pub.publish(ack);

    loop_rate.sleep();
  }

  return 0;
}
ros::Publisher ack_pub;
