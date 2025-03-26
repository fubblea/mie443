#include "ros/console.h"
#include "ros/init.h"
#include <contest3/contest3.h>
#include <contest3/state.h>

void RobotState::updateState(float secondsElapsed, bool contestMode) {
  switch (this->currState) {
  case State::START: {
    ROS_INFO("IT BEGINS");
    this->sc.playWave(SOUND_PATHS + "sound.wav");
    ROS_INFO("Did the sound play");
    ros::Duration(0.5).sleep();

    setState(State::FOLLOWING);
    break;
  }

  case State::FOLLOWING: {
    if (this->checkBumper() == BumperHit::NOTHING) {
      ROS_INFO("Bumper is clean, following you!");
      setVelCmd(this->follow_cmd);
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
      setState(State::FOLLOWING);
    }

    break;
  }

  case State::END: {
    ros::shutdown();
    break;
  }
  }
}
