#include "geometry_msgs/Twist.h"
#include "ros/console.h"
#include "ros/init.h"
#include <contest3/contest3.h>
#include <contest3/state.h>

State findFollowState(geometry_msgs::Twist follow_cmd) {
  if (follow_cmd.linear.x > 0) {
    return (State::FOLLOW_AHEAD);
  } else if (follow_cmd.linear.x < 0) {
    return (State::FOLLOW_BACK);
  } else {
    return (State::LOST);
  }
}

void RobotState::updateState(float secondsElapsed, bool contestMode) {
  switch (this->currState) {
  case State::START: {
    ROS_INFO("IT BEGINS");
    this->sc.playWave(SOUND_PATHS + "sound.wav");
    ROS_INFO("Did the sound play");
    ros::Duration(0.5).sleep();

    setState(findFollowState(this->follow_cmd));
    break;
  }

  case State::FOLLOW_AHEAD: {
    if (this->checkBumper() == BumperHit::NOTHING) {
      ROS_INFO("Bumper is clean, following you forward!");
      if (findFollowState(this->follow_cmd) == State::FOLLOW_AHEAD) {
        sc.playWave(SOUND_PATHS + "Happy.wav");
        ros::Duration(3.0).sleep();
        setVelCmd(this->follow_cmd);
      } else {
        setState(findFollowState(this->follow_cmd));
      }
    } else {
      ROS_INFO("Seems like a bumper is hit. Switching State");
      setState(State::IM_HIT);
    }

    break;
  }

  case State::FOLLOW_BACK: {
    if (this->checkBumper() == BumperHit::NOTHING) {
      ROS_INFO("Bumper is clean, following you back!");
      if (findFollowState(this->follow_cmd) == State::FOLLOW_BACK) {
        sc.playWave(SOUND_PATHS + "Disgust.wav");
        ros::Duration(3.0).sleep();
        setVelCmd(this->follow_cmd);
      } else {
        setState(findFollowState(this->follow_cmd));
      }
    } else {
      ROS_INFO("Seems like a bumper is hit. Switching State");
      setState(State::IM_HIT);
    }

    break;
  }

  case State::LOST: {
    if (this->checkBumper() == BumperHit::NOTHING) {
      ROS_INFO("Bumper is clean, but I'm lostttt!");
      if (findFollowState(this->follow_cmd) == State::LOST) {
        sc.playWave(SOUND_PATHS + "Sadness.wav");
        setVelCmd(this->follow_cmd);
      } else {
        setState(findFollowState(this->follow_cmd));
      }
    } else {
      ROS_INFO("Seems like a bumper is hit. Switching State");
      setState(State::IM_HIT);
    }

    break;
  }

  case State::IM_HIT: {

    if (this->checkBumper() != BumperHit::NOTHING) {
      ROS_INFO("Im hit!");
      setVelCmd(0, 0);
      sc.playWave(SOUND_PATHS + "PAIN.wav");
    } else {
      ROS_INFO("Does not hurt, going back to following");
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
