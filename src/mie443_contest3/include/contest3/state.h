#pragma once

#include "contest3/boxes.h"
#include "contest3/contest2.h"
#include "contest3/imagePipeline.h"
#include "contest3/lidar.h"
#include "ros/console.h"
#include "ros/node_handle.h"
#include <contest2/robot_pose.h>
#include <cstdlib>
#include <string>
#include <unordered_map>
#include <vector>

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
  std::unordered_map<int, std::vector<float>>
      identifiedTags; // initialize hashmap to store detected images
  RobotPose homePose;

private:
  std::vector<RobotPose> poseHist;

public:
  // Getters/Setters
  void setState(State newState) {
    this->poseHist.push_back(this->currPose);
    this->currState = newState;
  }

  // Constructors
  RobotState(ros::NodeHandle n)
      : currPose(0, 0, 0), imagePipeline(n), homePose(0, 0, 0) {};

  bool backAway(float desiredDist);

  // State Machine
  void updateState(bool showView, float secondsElapsed);
};