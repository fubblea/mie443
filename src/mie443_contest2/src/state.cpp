#include "contest2/state.h"
#include "contest2/contest2.h"
#include "contest2/robot_pose.h"
#include "ros/console.h"
#include <vector>

RobotPose getBoxNavGoal(float xBox, float yBox, float phiBox,
                        float angleOffset) {

  float theta = phiBox + DEG2RAD(angleOffset);

  float xPos = xBox + BOX_FACING_OFFSET * std::cos(theta);
  float yPos = yBox + BOX_FACING_OFFSET * std::sin(theta);
  float phiPos = theta + DEG2RAD(180);

  RobotPose newPose(xPos, yPos, phiPos);
  return newPose;
}

void RobotState::genNavGoals(float angleOffset) {
  for (int boxIdx = 0; boxIdx < this->boxes.coords.size(); boxIdx++) {
    float xBox = boxes.coords[boxIdx][0];
    float yBox = boxes.coords[boxIdx][1];
    float phiBox = boxes.coords[boxIdx][2];

    RobotPose navGoal = getBoxNavGoal(xBox, yBox, phiBox, angleOffset);

    this->goalList.push_back(RobotGoal(boxIdx, navGoal));

    ROS_INFO("Generated nav goal: (%f, %f, %f) for box %i", navGoal.x,
             navGoal.y, navGoal.phi, boxIdx);
  }
}

/*
Normalize an angle to be in the range of [-180, +180]
*/
float normalizeAngle(float angle) {
  while (angle > 180)
    angle -= 360;
  while (angle < -180)
    angle += 360;

  return angle;
}
