#include "contest2/state.h"
#include "contest2/contest2.h"
#include "contest2/robot_pose.h"
#include "ros/console.h"
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <fstream>
#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>

RobotPose getBoxNavGoal(float xBox, float yBox, float phiBox,
                        float angleOffset) {

  float theta;
  if (CONTEST_MODE) {
    theta = DEG2RAD(phiBox) + DEG2RAD(angleOffset);
  } else {
    theta = phiBox + DEG2RAD(angleOffset);
  }

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
    if (std::fabs(error) <= (ANGLE_TOL * 5)) {
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

bool RobotState::moveToWall(float targetDist, float speed) {
  if (targetDist < MIN_WALL_DIST) {
    targetDist = MIN_WALL_DIST;
    ROS_WARN("Distance to obstacle is less than %fm. Setting Target "
             "Distance to: %f",
             targetDist, targetDist);
  }
  if (this->lidarScan.wallDist > targetDist) {
    ROS_INFO("Moving to wall. Distance: %f", this->lidarScan.wallDist);
    this->velCmd.setVelCmd(true, speed, 0);
    return false;
  } else {
    ROS_INFO("I'm at the wall. Distance: %f", this->lidarScan.wallDist);
    this->velCmd.setVelCmd(false, 0, 0);
    return true;
  }
}

int findMode(const std::vector<int> &nums) {
  // Return 0 if the vector is empty (or handle error as needed)
  if (nums.empty()) {
    return 0;
  }

  // Map to store frequency of each number
  std::unordered_map<int, int> frequency;
  for (int num : nums) {
    frequency[num]++;
  }

  // Find the mode by checking which number has the highest frequency
  int mode = nums[0];
  int maxCount = 0;
  for (const auto &entry : frequency) {
    if (entry.second > maxCount) {
      maxCount = entry.second;
      mode = entry.first;
    }
  }

  return mode;
}

std::string getFileName(int boxGuess) {
  if (boxGuess == -1) {
    return "Blank";
  } else {
    boost::filesystem::path p(TEMPLATE_FILES[boxGuess]);
    return p.stem().string();
  }
}

void RobotState::saveTagsToFile() {
  system("mkdir -p detected_tags");
  std::ofstream myfile("detected_tags/box_guesses.txt");

  // Create hashmap of box guesses
  std::unordered_map<int, std::vector<BoxMatch>> boxGuesses;

  for (RobotGoal goal : this->goalList) {
    if (goal.boxIdGuesses.size() > 0) {
      for (BoxMatch guess : goal.boxIdGuesses) {
        boxGuesses[goal.boxIdx].push_back(guess);
      }
    }
  }

  // Write to file
  for (int boxIdx = 0; boxIdx < this->boxes.coords.size(); boxIdx++) {
    // Print box location
    myfile << "Box at (" << boxes.coords[boxIdx][0] << ", "
           << boxes.coords[boxIdx][1] << ", " << boxes.coords[boxIdx][2] << ")";

    // Find best guess
    int bestGuess = findMode(boxGuesses[boxIdx]);
    myfile << " - Best guess: " << getFileName(bestGuess);

    // List all guesses
    myfile << " - All guesses: (";
    for (const int &value : boxGuesses[boxIdx]) {
      myfile << getFileName(value) << ", ";
    }
    myfile << ")";

    // End line
    myfile << std::endl;
  }

  myfile.close();
}
