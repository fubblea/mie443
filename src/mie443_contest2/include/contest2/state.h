#pragma once

#include "contest2/boxes.h"
#include "contest2/imagePipeline.h"
#include "ros/node_handle.h"
#include <contest2/robot_pose.h>
#include <vector>

enum State {
  START,
  SPIN,
  GOTO_GOAL,
  TAG_BOX,
  IM_LOST,
  END,
};

class RobotGoal {
public:
  int boxIdx;
  int boxIdGuess;
  RobotPose pose;

public:
  RobotGoal(int boxIdx, RobotPose pose)
      : pose(pose.x, pose.y, pose.phi), boxIdx(boxIdx) {};
};

class RobotState {
public:
  State currState = State::START;
  RobotPose currPose;
  std::vector<RobotGoal> goalList;
  Boxes boxes;
  ImagePipeline imagePipeline;

public:
  // Constructors
  RobotState(ros::NodeHandle n) : currPose(0, 0, 0), imagePipeline(n) {};

  // Other Functions
  void genNavGoals(float angleOffset);

  // State Machine
  void updateState(bool showView);
};
