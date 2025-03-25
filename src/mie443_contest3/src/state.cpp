#include "contest3/state.h"
#include "contest3/contest2.h"
#include "contest3/robot_pose.h"
#include "ros/console.h"
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <contest2/utils.h>
#include <fstream>
#include <ostream>
#include <unordered_map>
#include <vector>

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