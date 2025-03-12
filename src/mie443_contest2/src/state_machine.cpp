#include "contest2/contest2.h"
#include "contest2/imagePipeline.h"
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

    ROS_INFO("Gonna memorize the templates now");
    ROS_INFO("template 1: %s", TEMPLATE_FILES[0].c_str());
    this->imagePipeline.memorizeTemplates();

    setState(State::SPIN);
    break;
  }

  case State::SPIN: {
    ROS_INFO("You spin me right round baby right round like a record baby "
             "right round right round");
    if (doTurn(MAX_SPIN_ANGLE, poseHist.back().phi, false)) {
      ROS_INFO("finding my first goal");
      setState(State::GOTO_GOAL);
    }

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
      this->lostCount++;
      setState(State::IM_LOST);
    } else {
      ROS_INFO("Navigating was successful");
      this->lostCount = 0;
      setState(State::TAG_BOX);
    }

    break;
  }

  case State::TAG_BOX: {

    int currGuess = this->imagePipeline.getTemplateID(this->boxes, showView);

    this->goalList[0].boxIdGuesses.push_back(currGuess);

    ROS_INFO("Guess for box at (%f, %f, %f) is %i. Match found?: %i",
             this->boxes.coords[this->goalList[0].boxIdx][0],
             this->boxes.coords[this->goalList[0].boxIdx][1],
             this->boxes.coords[this->goalList[0].boxIdx][2], currGuess,
             currGuess != -1);

    sendGoalToBack(&this->goalList, 0);

    setState(State::GOTO_GOAL);

    break;
  }

  case State::IM_LOST: {
    ROS_WARN("IM LOSTTTTTTT AHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH");

    setState(State::GOTO_GOAL);
    break;

    if (this->lostCount <= MAX_LOST_COUNT) {
      ROS_WARN("Still lost, trying to get unlost. Lost count: %i",
               this->lostCount);

      if (doTurn(90, this->poseHist.back().phi, true)) {
        if (moveToWall(MIN_WALL_DIST + 0.1, MAX_LIN_VEL)) {
          setState(State::GOTO_GOAL);
        }
      }
    } else {
      ROS_ERROR("Can't get unlost, skipping goal");
      sendGoalToBack(&this->goalList, 0);
      setState(State::GOTO_GOAL);
      this->lostCount = 0;
    }

    break;
  }

  default: {
    ROS_ERROR("Undefined state: %i", this->currState);
    setState(State::IM_LOST);
    break;
  }
  }
}
