#include "contest2/state.h"
#include "contest2/contest2.h"
#include "ros/console.h"
#include <vector>

RobotPose getBoxNavGoal(float xBox, float yBox, float phiBox,
                        float angleOffset) {
  float xPos = xBox + BOX_FACING_OFFSET * std::cos(phiBox);
  float yPos = yBox + BOX_FACING_OFFSET * std::sin(phiBox);
  float phiPos = phiBox + DEG2RAD(180);

  RobotPose newPose(xPos, yPos, phiPos);
  return newPose;
}

std::vector<RobotGoal> RobotState::genNavGoals() {
  std::vector<RobotGoal> navGoals;
  for (int boxIdx = 0; boxIdx < this->boxes.coords.size(); boxIdx++) {
    float xBox = boxes.coords[boxIdx][0];
    float yBox = boxes.coords[boxIdx][1];
    float phiBox = boxes.coords[boxIdx][2];

    navGoals.push_back(RobotGoal(
        boxIdx, getBoxNavGoal(xBox, yBox, phiBox, 0))); // Straight ahead
    navGoals.push_back(
        RobotGoal(boxIdx, getBoxNavGoal(xBox, yBox, phiBox,
                                        BOX_ANGLE_OFFSET))); // Off to right
    navGoals.push_back(
        RobotGoal(boxIdx, getBoxNavGoal(xBox, yBox, phiBox,
                                        -BOX_ANGLE_OFFSET))); // Off to left

    ROS_INFO("Generated nav goals for box %i", boxIdx);
  }

  return navGoals;
}
