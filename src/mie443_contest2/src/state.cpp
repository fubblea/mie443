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

bool RobotState::doTurn(float relativeTarget, float reference, bool quick) {
  bool isComplete = false;

  // Current angle is relative to the start of the state
  float targetBearing = normalizeAngle(RAD2DEG(reference) + relativeTarget);
  float error = normalizeAngle(RAD2DEG(this->currPose.phi) - targetBearing);

  ROS_INFO("Turning to %f deg. Delta: %f (%f) deg", targetBearing, error,
           RAD2DEG(this->currPose.phi));
  if (std::fabs(error) > ANGLE_TOL) {
    float turn_speed = MAX_ANG_VEL;
    // Slow down turning if we are close to the target
    if (std::fabs(error) <= (ANGLE_TOL * 2)) {
      turn_speed = MIN_ANG_VEL;
    }

    float turn_vel;
    if (quick) {
      turn_vel = (error < 0) ? turn_speed : -turn_speed;
    } else {
      turn_vel = (relativeTarget > 0) ? turn_speed : -turn_speed;
    }
    this->velCmd.setVelCmd(true, 0, turn_vel);
  } else {
    this->velCmd.setVelCmd(false, 0, 0);
    isComplete = true;
  }

  return isComplete;
}

bool RobotState::backAway(float desiredDist) {
  float current_x = this->poseHist.back().x;
  float current_y = this->poseHist.back().y;

  float distMoved = sqrt(powf((this->currPose.x - current_x), 2) +
                         powf((this->currPose.y - current_y), 2));

  if (distMoved < desiredDist) {
    this->velCmd.setVelCmd(true, -SLOW_LIN_VEL, 0);
    return false;
  } else {
    this->velCmd.setVelCmd(false, 0, 0);
    return true;
  }
}
