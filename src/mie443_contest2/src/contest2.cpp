#include "navigation.h"
#include "robot_pose.h"
#include "ros/console.h"
#include <cmath>
#include <contest2.h>

RobotPose moveFacingBox(float xBox, float yBox, float phiBox) {
  float xPos = xBox + BOX_FACING_OFFSET * std::cos(phiBox);
  float yPos = yBox + BOX_FACING_OFFSET * std::sin(phiBox);
  float phiPos = phiBox + DEG2RAD(180);

  RobotPose newPose(xPos, yPos, phiPos);
  return newPose;
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

  // Setup ROS.
  ros::init(argc, argv, "contest2");
  ros::NodeHandle n;
  // Robot pose object + subscriber.
  RobotPose robotPose(0, 0, 0);
  ros::Subscriber amclSub =
      n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
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
  // Initialize image objectand subscriber.
  ImagePipeline imagePipeline(n);

  // contest count down timer
  std::chrono::time_point<std::chrono::system_clock> start;
  start = std::chrono::system_clock::now();
  uint64_t secondsElapsed = 0;

  int boxIdx = 0;
  int numBoxs = boxes.coords.size();

  bool firstSpin = true;

  // Execute strategy.
  while (ros::ok() && secondsElapsed <= 300) {
    ros::spinOnce();

    ROS_INFO("Current pos: (%f, %f, %f)", robotPose.x, robotPose.y,
             robotPose.phi);
    ROS_INFO("Box target: (%f, %f, %f)", boxes.coords[boxIdx][0],
             boxes.coords[boxIdx][1], boxes.coords[boxIdx][2]);

    RobotPose navGoal =
        moveFacingBox(boxes.coords[boxIdx][0], boxes.coords[boxIdx][1],
                      boxes.coords[boxIdx][2]);

    // Let the callbacks update once
    if (firstSpin) {
      if (robotPose.x != 0 || robotPose.y != 0 || robotPose.phi != 0) {
        ROS_INFO("First spin done");
        firstSpin = false;
      } else {
        ROS_INFO("Still waiting for first spin");
      }

    } else {
      // Navigate to box poses
      ROS_INFO("Nav goal: (%f, %f, %f)", navGoal.x, navGoal.y, navGoal.phi);
      Navigation::moveToGoal(navGoal.x, navGoal.y, navGoal.phi);

      boxIdx++;
      if (boxIdx == numBoxs) {
        boxIdx = 0;
      }

      imagePipeline.getTemplateID(boxes, showView);
    }

    ros::Duration(0.01).sleep();
  }
  return 0;
}
