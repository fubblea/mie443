#include "state.h"
#include "contest1.h"

// Angle tolerance
const float ANGLE_TOL = 1; // deg

void robotState::update() {
  if (currState == State::START) {
    setVelCmd(0, 0);
    setState(State::SPIN);
  } else if (currState == State::SPIN) {
    if (doTurn(359, stateRef.yaw, false)) {
      setState(State::FIND_WALL);
    }
  } else if (currState == State::FIND_WALL) {
    // First, turn left 90 deg
    if (doTurn(90, stateRef.yaw, true)) {
      // Then, move up to the wall
      if (moveToWall(0.2)) {
        setState(State::FIND_WALL);
      }
    }
  } else {
    setVelCmd(0, 0);
    ROS_INFO("Program End");
  }
}

/*
Normalize an angle to be in the range of [-180, +180]
*/
float normalizeAngle(float angle) {
  while (angle > 180)
    angle -= 360;
  while (angle < -180)
    angle += 360;

  return angle;
}

bool robotState::doTurn(float relativeTarget, float reference, bool quick) {
  bool isComplete = false;

  // Current angle is relative to the start of the state
  float targetBearing = normalizeAngle(reference + relativeTarget);
  float error = normalizeAngle(stateVars.yaw - targetBearing);

  if (std::fabs(error) > ANGLE_TOL) {
    ROS_INFO("Turning to %f deg. Error: %f (%f) deg", targetBearing, error,
             stateVars.yaw);

    float turn_speed = MAX_ANG_VEL;
    // Slow down turning if we are close to the target
    if (std::fabs(error) <= (ANGLE_TOL * 5)) {
      turn_speed = turn_speed / 5;
    }

    float turn_vel;
    if (quick) {
      turn_vel = (error < 0) ? turn_speed : -turn_speed;
    } else {
      turn_vel = (relativeTarget > 0) ? turn_speed : -turn_speed;
    }
    setVelCmd(turn_vel, 0);
  } else {
    isComplete = true;
  }

  return isComplete;
}

bool robotState::moveToWall(float targetDist, float speed) {
  if (stateVars.wallDist > targetDist) {
    ROS_INFO("Moving to wall. Distance: %f", stateVars.wallDist);
    setVelCmd(0, speed);
    return false;
  } else {
    setVelCmd(0, 0);
    return true;
  }
}

Vel::Vel(float angular, float linear) {
  angular = angular;
  linear = linear;
}
