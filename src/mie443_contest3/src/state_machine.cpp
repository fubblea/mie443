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
      setVelCmd(0, 0);
      setState(State::IM_HIT);
    }

    break;
  }

  case State::IM_HIT: {
    // TODO:
    break;
  }

  case State::END: {
    ros::shutdown();
    break;
  }
  }
}
