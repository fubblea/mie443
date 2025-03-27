#include "geometry_msgs/Twist.h"
#include "ros/console.h"
#include "ros/init.h"
#include <atomic>
#include <contest3/contest3.h>
#include <contest3/state.h>
#include <thread>

State findFollowState(geometry_msgs::Twist follow_cmd) {
  if (follow_cmd.linear.x > 0) {
    return (State::FOLLOW_AHEAD);
  } else if (follow_cmd.linear.x < 0) {
    return (State::FOLLOW_BACK);
  } else {
    return (State::LOST);
  }
}

std::atomic<bool> soundDone(true);

void callAsyncSound(RobotState &state, std::string filePath) {
  if (soundDone.load()) {
    ROS_INFO("Sound is done");

    soundDone.store(false);
    std::thread th_sound(&RobotState::playSound, state, filePath, &soundDone);
    th_sound.detach();
  } else {
    ROS_INFO("Waiting for soundDone");
  }
}

void RobotState::updateState(float secondsElapsed, bool contestMode) {
  switch (this->currState) {
  case State::START: {
    ROS_INFO("IT BEGINS");

    callAsyncSound(*this, SOUND_PATHS + "sound.wav");

    setState(findFollowState(this->follow_cmd));

    break;
  }

  case State::FOLLOW_AHEAD: {
    if (this->checkEvents() == EventStatus::BUMPER_HIT) {
      ROS_INFO("Seems like a bumper is hit. Switching State");
      setState(State::IM_HIT);
    } else if (this->checkEvents() == EventStatus::CLIFF_HIT) {
      ROS_INFO("I been picked up");
      setState(State::PICKED_UP);
    } else {

      ROS_INFO("Following forward");
      if (findFollowState(this->follow_cmd) == State::FOLLOW_AHEAD) {
        setVelCmd(this->follow_cmd);
      } else {
        setState(findFollowState(this->follow_cmd));
      }
    }

    break;
  }

  case State::FOLLOW_BACK: {
    if (this->checkEvents() == EventStatus::BUMPER_HIT) {
      ROS_INFO("Seems like a bumper is hit. Switching State");
      setState(State::IM_HIT);
    } else if (this->checkEvents() == EventStatus::CLIFF_HIT) {
      ROS_INFO("I been picked up");
      setState(State::PICKED_UP);
    } else {

      ROS_INFO("Following backward");
      if (findFollowState(this->follow_cmd) == State::FOLLOW_BACK) {
        sc.playWave(SOUND_PATHS + "Disgust.wav");
        ros::Duration(0.5).sleep();
        setVelCmd(this->follow_cmd);
      } else {
        setState(findFollowState(this->follow_cmd));
      }
    }

    break;
  }

  case State::LOST: {
    if (this->checkEvents() == EventStatus::BUMPER_HIT) {
      ROS_INFO("Seems like a bumper is hit. Switching State");
      setState(State::IM_HIT);
    } else if (this->checkEvents() == EventStatus::CLIFF_HIT) {
      ROS_INFO("I been picked up");
      setState(State::PICKED_UP);
    } else {

      ROS_INFO("Bumper is clean, but I'm lostttt!");
      if (findFollowState(this->follow_cmd) == State::LOST) {
        sc.playWave(SOUND_PATHS + "Sadness.wav");
        ros::Duration(0.5).sleep();
        setVelCmd(this->follow_cmd);
      } else {
        setState(findFollowState(this->follow_cmd));
      }
    }

    break;
  }

  case State::IM_HIT: {
    if (this->checkEvents() == EventStatus::BUMPER_HIT) {
      ROS_INFO("Im hit!");
      setVelCmd(0, 0);
      sc.playWave(SOUND_PATHS + "PAIN.wav");
      ros::Duration(0.5).sleep();
    } else {
      ROS_INFO("Does not hurt, going back to following");
      setState(findFollowState(this->follow_cmd));
    }

    break;
  }

  case State::PICKED_UP: {
    if (this->checkEvents() == EventStatus::CLIFF_HIT) {
      ROS_INFO("PUT ME DOWN MF!");
      setVelCmd(0, 0);
      sc.playWave(SOUND_PATHS + "Happy.wav");
      ros::Duration(0.5).sleep();
    } else {
      ROS_INFO("Back down, going back to following");
      setState(findFollowState(this->follow_cmd));
    }

    break;
  }

  case State::END: {
    ros::shutdown();
    break;
  }
  }
}
