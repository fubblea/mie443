#pragma once

// INTERNAL HEADER FILES
#include <contest2/robot_pose.h>
#include <vector>

class RobotState {
public:
  RobotPose currPose;
  std::vector<RobotPose> goalList;

public:
  // Constructors
  RobotState() : currPose(0, 0, 0) {};
};
