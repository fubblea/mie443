#include "contest2/contest2.h"
#include "contest2/navigation.h"
#include "ros/console.h"
#include <algorithm>
#include <contest2/state.h>
#include <vector>

void sendGoalToBack(std::vector<RobotGoal> *goalList, int goalIdx) {
  if (!goalList || goalIdx >= goalList->size()) {
    ROS_ERROR("sendGoalToBack(): Invalid arguments");
    return;
  }

  ROS_INFO("Sending goal %i to back of the goalList", goalIdx);
  std::rotate(goalList->begin() + goalIdx, goalList->begin() + goalIdx + 1,
              goalList->end());
}

void RobotState::updateState(bool showView) {
  switch (this->currState) {

  case State::START: {
    ROS_INFO("Initialization. Generating navigation goals");

    genNavGoals(0);
    genNavGoals(BOX_ANGLE_OFFSET);
    genNavGoals(-BOX_ANGLE_OFFSET);

    this->currState = State::GOTO_GOAL;
    break;
  }

  case State::SPIN: {
    ROS_INFO("First half-spin");
    Navigation::moveToGoal(this->currPose.x, this->currPose.y,
                           this->currPose.phi + DEG2RAD(180));

    ROS_INFO("Second half-spin");
    Navigation::moveToGoal(this->currPose.x, this->currPose.y,
                           this->currPose.phi);

    this->currState = State::GOTO_GOAL;

    break;
  }

  case State::GOTO_GOAL: {
    ROS_INFO("Navigating to goal: (%f, %f, %f)", this->goalList[0].pose.x,
             this->goalList[0].pose.y, this->goalList[0].pose.phi);

    bool moveSuccess = Navigation::moveToGoal(this->goalList[0].pose.x,
                                              this->goalList[0].pose.y,
                                              this->goalList[0].pose.phi);

    if (!moveSuccess) {
      ROS_ERROR("Navigation was not successful!");
      this->currState = State::IM_LOST;
    } else {
      ROS_INFO("Navigating was successful");
      this->currState = State::TAG_BOX;
    }

    break;
  }

  case State::TAG_BOX: {
    this->goalList[0].boxIdGuess =
        this->imagePipeline.getTemplateID(this->boxes, showView);

    ROS_INFO("Guess for box at (%f, %f, %f) is %i",
             this->boxes.coords[this->goalList[0].boxIdx][0],
             this->boxes.coords[this->goalList[0].boxIdx][1],
             this->boxes.coords[this->goalList[0].boxIdx][2],
             this->goalList[0].boxIdGuess);

    sendGoalToBack(&this->goalList, 0);

    this->currState = State::GOTO_GOAL;

    break;
  }

  case State::IM_LOST: {
    // TODO: Random walk
    sendGoalToBack(&this->goalList, 0);
    this->currState = State::GOTO_GOAL;

    break;
  }

  default: {
    ROS_ERROR("Undefined state: %i", this->currState);
    this->currState = State::IM_LOST;
    break;
  }
  }
}
