#include "contest3/state.h"
#include "contest3/callbacks.h"
#include "contest3/contest3.h"
#include "ros/console.h"
#include <atomic>
#include <chrono>
#include <thread>

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

void RobotState::playSound(std::string filePath, int soundLength,
                           std::atomic<bool> *soundDone) {
  ROS_INFO("Async Play Sound: %s", filePath.c_str());
  this->sc.stopAll();
  this->sc.playWave(filePath);
  std::this_thread::sleep_for(std::chrono::milliseconds(soundLength));
}

float normalizeAngle(float angle) {
  while (angle > 180)
    angle -= 360;
  while (angle < -180)
    angle += 360;

  return angle;
}

bool RobotState::doTurn(float relativeTarget, float reference, bool quick) {
  bool isComplete = false;

  // Current angle is relative to the start of the state
  float targetBearing = normalizeAngle(reference + relativeTarget);
  float error = normalizeAngle(currPose.yaw - targetBearing);

  if (std::fabs(error) > ANGLE_TOL) {
    ROS_INFO("Turning to %f deg. Delta: %f (%f) deg", targetBearing, error,
             currPose.yaw);

    float turn_speed = MAX_ANGLE_VEL;
    // Slow down turning if we are close to the target
    if (std::fabs(error) <= (ANGLE_TOL * 2)) {
      turn_speed = MIN_ANGLE_VEL;
    }

    float turn_vel;
    if (quick) {
      turn_vel = (error < 0) ? turn_speed : -turn_speed;
    } else {
      turn_vel = (relativeTarget > 0) ? turn_speed : -turn_speed;
    }
    setVelCmd(0, turn_vel);
  } else {
    isComplete = true;
  }

  return isComplete;
}