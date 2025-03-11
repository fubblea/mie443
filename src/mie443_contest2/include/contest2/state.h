#pragma once

#include "contest2/boxes.h"
#include "contest2/contest2.h"
#include "contest2/imagePipeline.h"
#include "contest2/lidar.h"
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

/**
 * @class RobotVelCmd
 * @brief A struct representing a robot velocity command, comprising of a linear
 * and angular velocity
 *
 */
class RobotVelCmd {
public:
  // If true, the robot is in manual command mode
  bool cmdActive = false;

  // Linear velocity command
  float linVel;

  // Angular velocity command
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

/**
 * @class RobotState
 * @brief A class representing the current state of the robot and the state
 * variables
 *
 */
class RobotState {
public:
  State currState;
  RobotPose currPose;
  RobotVelCmd velCmd;
  std::vector<RobotGoal> goalList;
  Boxes boxes;
  ImagePipeline imagePipeline;
  int lostCount = 0;
  LidarScan lidarScan;

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
  bool backAway(float desiredDist);
  bool moveToWall(float targetDist, float speed);

  // Other Functions
  void genNavGoals(float angleOffset);

  // State Machine
  void updateState(bool showView);
};
