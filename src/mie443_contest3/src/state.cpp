#include "contest3/state.h"

BumperHit RobotState::checkBumper() {
  bool bumperPressed = false;
  int bumperNum = 0;

  for (uint32_t bumperID = 0; bumperID < 3; ++bumperID) {
    bumperPressed |=
        (this->bumper[bumperID] == kobuki_msgs::BumperEvent::PRESSED);
    bumperNum = bumperID;
  }
  if (!bumperPressed) {
    this->bumperHit = BumperHit::NOTHING;
  } else {
    if (bumperNum == 0) {
      ROS_INFO("I'm hit at left!");
      this->bumperHit = BumperHit::LEFT;
    } else if (bumperNum == 1) {
      ROS_INFO("I'm hit at center!");
      this->bumperHit = BumperHit::CENTER;
    } else if (bumperNum == 2) {
      ROS_INFO("I'm hit at right!");
      this->bumperHit = BumperHit::RIGHT;
    }
  }

  return this->bumperHit;
}

EventStatus RobotState::checkEvents() {
  if (this->checkBumper() != NOTHING) {
    return EventStatus::BUMPER_HIT;
  } else if (this->cliffEvent.state == 1) {
    return EventStatus::CLIFF_HIT;
  } else {
    return EventStatus::ALL_GOOD;
  }
}
