#pragma once

#include <geometry_msgs/PoseWithCovarianceStamped.h>

class RobotPose {
public:
  float x;
  float y;
  float phi;

public:
  // Constructors
  RobotPose(float x, float y, float phi);

  // Callbacks
  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);
};
