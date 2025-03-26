#include "ros/console.h"
#include "ros/init.h"
#include <contest3/state.h>

void RobotState::updateState(float secondsElapsed, bool contestMode) {
  switch (this->currState) {
  case State::START: {
    this->sc.playWave(SOUND_PATHS + "sound.wav");
    ros::Duration(0.5).sleep();

    setState(State::FOLLOWING);
    break;
  }

  case State::FOLLOWING: {
    if (this->checkBumper() == BumperHit::NOTHING) {
      setVelCmd(this->follow_cmd);
    } else {
      setState(State::IM_HIT);
    }

    break;
  }

  case State::IM_HIT: {
    ROS_INFO("Im hit!");
    setVelCmd(0, 0);
    sc.playWave(std::string("src/mie443_contest3/sounds") + "PAIN.wav");

    break;
  }

  case State::END: {
    ros::shutdown();
    break;
  }
  }
}
