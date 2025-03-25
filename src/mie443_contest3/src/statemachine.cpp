#include "contes3/imageTransporter.hpp"
#include "contest3/contest3.h"
#include "contest3/header.h"
#include "contest3/state.h"

void RobotState::updateState(bool showView, float secondsElapsed) {
  if (secondsElapsed > GO_HOME_TIME) {
    ROS_WARN("Time to go home!! Setting state to GO_HOME");
    setState(State::GO_HOME);
  } else {
    ROS_INFO("We still have time: %f", secondsElapsed);
  }

  switch (this->currState) {

  case State::START: {

    setState(State::SPIN);
    break;
  }

  case State::SPIN: {
    ROS_INFO("You spin me right round baby right round like a record baby "
             "right round right round");
    // if (doTurn(MAX_SPIN_ANGLE, poseHist.back().phi, false)) {
    //   ROS_INFO("Remembering home pose");
    //   this->homePose = currPose;

    //   ROS_INFO("finding my first goal");
    //   setState(State::GOTO_GOAL);
    // }

    ROS_INFO("Remembering home pose");
    this->homePose = currPose;

    ROS_INFO("finding my first goal");
    setState(State::GOTO_GOAL);

    break;
  }
  case State::END: {
    ROS_INFO("Graceful exit");
    ros::shutdown();

    break;
  }
  }
}