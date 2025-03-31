#include "geometry_msgs/Twist.h"
#include "ros/console.h"
#include "ros/init.h"
#include <atomic>
#include <contest3/contest3.h>
#include <contest3/state.h>
#include <opencv2/opencv.hpp>
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

void callAsyncThread(RobotState &state, std::string filePath,
                     std::string imgPath, int soundLength = 3000) {
  if (soundDone.load()) {
    ROS_INFO("Sound playing: %s", filePath.c_str());
    soundDone.store(false);
    std::thread th_sound(&RobotState::playSound, state, filePath, soundLength,
                         &soundDone);
    th_sound.detach();

    ROS_INFO("image path is: %s", imgPath.c_str());
    cv::Mat img = cv::imread(imgPath);
    imshow("displayed image", img);
    cv::waitKey(50);

  } else {
    ROS_INFO("Waiting for soundDone");
  }
}

void RobotState::updateState(float secondsElapsed, bool contestMode) {
  switch (this->currState) {
  case State::START: {
    ROS_INFO("IT BEGINS");

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
        callAsyncThread(*this, SOUND_PATHS + "Disgust.wav",
                        IMG_PATHS + "Disgust1.jpeg", 4000);
        setVelCmd(this->follow_cmd);
      } else {
        cv::destroyAllWindows();
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
        callAsyncThread(*this, SOUND_PATHS + "Sadness.wav",
                        IMG_PATHS + "Sadness.jpeg", 2000);
        if (doTurn(MAX_SPIN_ANGLE, stateHist.back().yaw, true)) {
          ROS_INFO("Looking for you");
          setState(findFollowState(this->follow_cmd));
        }

      } else {
        cv::destroyAllWindows();
        setState(findFollowState(this->follow_cmd));
      }
    }

    break;
  }

  case State::IM_HIT: {
    if (this->checkEvents() == EventStatus::BUMPER_HIT) {
      ROS_INFO("Im hit!");
      setVelCmd(0, 0);
      callAsyncThread(*this, SOUND_PATHS + "Anger.wav",
                      IMG_PATHS + "Anger.jpeg", 3000);
    } else {
      ROS_INFO("Does not hurt, going back to following");
      cv::destroyAllWindows();
      setState(findFollowState(this->follow_cmd));
    }

    break;
  }

  case State::PICKED_UP: {
    if (this->checkEvents() == EventStatus::CLIFF_HIT) {
      ROS_INFO("PUT ME DOWN MF!");
      setVelCmd(0, 0);
      callAsyncThread(*this, SOUND_PATHS + "Fear.wav", IMG_PATHS + "Fear.jpeg",
                      500);
    } else {
      ROS_INFO("Back down, going back to following");
      cv::destroyAllWindows();
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
