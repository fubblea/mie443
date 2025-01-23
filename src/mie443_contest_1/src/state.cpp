#include "state.h"
#include "contest1.h"

// Angle tolerance
const float ANGLE_TOL = 0.5; // deg

void robotState::update() {
  if (currState == State::START) {
    setVelCmd(0, 0);
    setState(State::SPIN);
  } else if (currState == State::SPIN) {
    if (doTurn(359, stateRef.yaw)) {
      setState(State::END);
    }
  } else if (currState == State::END) {
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

bool robotState::doTurn(float relativeTarget, float reference = 0) {
  bool isComplete = false;

  // Current angle is relative to the start of the state
  float targetBearing = normalizeAngle(reference + relativeTarget);
  float error = std::fabs(normalizeAngle(stateVars.yaw - targetBearing));

  if (error > ANGLE_TOL) {
    ROS_INFO("Turning to %f deg. Error: %f (%f) deg", targetBearing, error,
             stateVars.yaw);

    float turn_vel = (relativeTarget > 0) ? MAX_ANG_VEL : -MAX_ANG_VEL;
    setVelCmd(turn_vel, 0);
  } else {
    isComplete = true;
  }
}

Vel::Vel(float angular, float linear) {
  angular = angular;
  linear = linear;
}
