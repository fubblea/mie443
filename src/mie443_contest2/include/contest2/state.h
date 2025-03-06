#pragma once

#include "contest2/boxes.h"
#include "contest2/contest2.h"
#include "contest2/imagePipeline.h"
#include "ros/console.h"
#include "ros/node_handle.h"
#include <contest2/robot_pose.h>
#include <cstdlib>
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

class RobotVelCmd {
public:
  bool cmdActive = false;
  float linVel;
  float angVel;

public:
  void setCmdActive(bool cmdActive) {
    this->cmdActive = cmdActive;
    if (cmdActive) {
      ROS_WARN("Manual motor command is active!");
    } else {
      ROS_WARN("Manual motor command deactivated");
    }
  }

  void setVelCmd(bool cmdActive, float linVel, float angVel) {
    setCmdActive(cmdActive);
    this->linVel = (std::fabs(linVel) > MAX_LIN_VEL) ? MAX_LIN_VEL : linVel;
    this->angVel = (std::fabs(angVel) > MAX_ANG_VEL) ? MAX_ANG_VEL : angVel;
  }
};

class RobotState {
public:
  State currState = State::START;
  RobotPose currPose;
  RobotVelCmd velCmd;
  std::vector<RobotGoal> goalList; // (boxIdx, navGoal)
  Boxes boxes;
  ImagePipeline imagePipeline;

private:
  std::vector<RobotPose> poseHist;

public:
  // Getters/Setters
  void setState(State newState) {
    this->poseHist.push_back(this->currPose);
    this->currState = newState;
  }

  // Constructors
  RobotState(ros::NodeHandle n) : currPose(0, 0, 0), imagePipeline(n) {};

  // Movement Functions
  bool doTurn(float relativeTarget, float reference, bool quick);

  // Other Functions
  void genNavGoals(float angleOffset);

  // State Machine
  void updateState(bool showView);
};
