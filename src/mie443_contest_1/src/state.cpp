#include "state.h"
#include "contest1.h"

// Angle tolerance
const float ANGLE_TOL = 1; // deg

Vel::Vel(float angular, float linear) {
  angular = angular;
  linear = linear;
}

void robotState::updateVisitedPos() {
  // Add the current position to the vec of already visited positions
  std::tuple<float, float> currPos =
      std::make_tuple(stateVars.posX, stateVars.posY);
  int cnt = std::count(stateVars.visitedPos.begin(), stateVars.visitedPos.end(),
                       currPos);

  // Add only if not previously added
  if (cnt < 0) {
    stateVars.visitedPos.push_back(currPos);
    ROS_INFO("Added (%f, %f) to visitedPos", std::get<0>(currPos),
             std::get<1>(currPos));
  }
}

// ===================STATE MACHINE =============================

void robotState::update() {
  switch (currState) {

  // Program start
  case State::START:
    setVelCmd(0, 0);

    if (checkBumper() != BumperHit::NOTHING) {
      setState(State::IM_HIT);
    } else {
      setState(State::SPIN);
    }

    break;

  // Spinning around
  case State::SPIN:
    if (doTurn(MAX_SPIN_ANGLE, stateHist.back().yaw, false)) {
      setState(State::THINK);
    }
    break;

  // Think about what to do
  case State::THINK:
    ROS_INFO("Contemplating life");
    if (checkBumper() == BumperHit::NOTHING) {
      if (stateVars.wallDist >= 0.6) {
        ROS_INFO("Distance to wall is %f m, going FAST", stateVars.wallDist);
        setState(State::IM_SPEED);
      } else {
        // ROS_INFO("Distance to wall is %f m, going SLOW", stateVars.wallDist);
        // setState(State::IM_SLOW);
        ROS_INFO("I am close to wall, time to turn");
        setState(State::IM_HIT);
      }
    } else {
      ROS_INFO("I am close to wall, time to turn");
      setState(State::IM_HIT);
    }
    break;

  // Speed to the wall
  case State::IM_SPEED:
    ROS_INFO("Speed to the wall");
    if (moveToWall(0, MAX_LIN_VEL)) {
      setState(State::THINK);
    }
    break;

  // Slowly creep to wall
  case State::IM_SLOW:
    ROS_INFO("Creeping to the wall. Distance to wall is %f m",
             stateVars.wallDist);
    if (moveTilBumped(SLOW_LIN_VEL)) {
      setState(State::THINK);
    }
    break;

  case State::IM_HIT:
    ROS_INFO("I was hit at bumper %i. Reorienting.",
             stateHist.back().bumperHit);
    if (backAway(0.1)) {
      bool allSorted = false;

      // Check if this position has been preivously visited
      if (checkVisit(stateHist.back().posX, stateHist.back().posY), 1) {
        ROS_INFO("I've been here before");
        allSorted = doTurn(-85, stateHist.back().yaw, true);
      } else {
        ROS_INFO("I've NOT been here before");
        allSorted = doTurn(95, stateHist.back().yaw, true);
      }

      if (allSorted) {
        setState(State::SPIN);
      }
    }
    break;

  // Program end
  case State::END:
    setVelCmd(0, 0);
    ROS_INFO("Aight I'm outta here");
    ros::shutdown();
    break;
  }

  updateVisitedPos();
}

// ===================STATE MACHINE =============================

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
    ROS_INFO("Turning to %f deg. Delta: %f (%f) deg", targetBearing, error,
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
  if (targetDist < 0.5) {
    targetDist = 0.5;
    ROS_WARN("Distance to obstacle is less than %fm. Setting Target "
             "Distance to: %f",
             targetDist, targetDist);
  }
  if (stateVars.wallDist > targetDist) {
    ROS_INFO("Moving to wall. Distance: %f", stateVars.wallDist);
    setVelCmd(0, speed);
    return false;
  } else {
    ROS_INFO("I'm at the wall. Distance: %f", stateVars.wallDist);
    setVelCmd(0, 0);
    return true;
  }
}

BumperHit robotState::checkBumper() {
  bool bumperPressed = false;
  int bumperNum = 0;

  for (uint32_t bumperID = 0; bumperID < 3; ++bumperID) {
    bumperPressed |=
        (stateVars.bumper[bumperID] == kobuki_msgs::BumperEvent::PRESSED);
    bumperNum = bumperID;
  }
  if (!bumperPressed) {
    stateVars.bumperHit = BumperHit::NOTHING;
  } else {
    if (bumperNum == 0) {
      ROS_INFO("I'm hit at left!");
      stateVars.bumperHit = BumperHit::LEFT;
    } else if (bumperNum == 1) {
      ROS_INFO("I'm hit at center!");
      stateVars.bumperHit = BumperHit::CENTER;
    } else if (bumperNum == 2) {
      ROS_INFO("I'm hit at right!");
      stateVars.bumperHit = BumperHit::RIGHT;
    }
  }

  return stateVars.bumperHit;
}

bool robotState::backAway(float desiredDist) {
  float current_x = stateHist.back().posX;
  float current_y = stateHist.back().posY;

  float distMoved = sqrt(powf((stateVars.posX - current_x), 2) +
                         powf((stateVars.posY - current_y), 2));

  if (distMoved < desiredDist) {
    setVelCmd(0, -SLOW_LIN_VEL);
    return false;
  } else {
    setVelCmd(0, 0);
    return true;
  }
}

bool robotState::checkVisit(float posX, float posY, float tol) {
  // Using the second to last history state to avoid points overlap
  StateVars reference = stateHist.at(stateHist.size() - 2);

  for (int idx = 0; idx < reference.visitedPos.size(); idx++) {
    float checkX = std::get<0>(reference.visitedPos[idx]);
    float checkY = std::get<1>(reference.visitedPos[idx]);

    // Create a box around the point with the specified tolerance
    float xMin = (checkX > tol) ? checkX - tol : 0;
    float xMax = checkX + tol;
    float yMin = (checkY > tol) ? checkY - tol : 0;
    float yMax = checkY + tol;

    // Check if the reference position is in this box
    if (posX >= xMin && posX <= xMax && posY >= yMin && posY <= yMax) {
      return true;
    }
  }

  return false;
}

bool robotState::moveTilBumped(float vel) {
  switch (checkBumper()) {

  case BumperHit::NOTHING:
    setVelCmd(0, vel);
    ROS_INFO("Moving till hit at %f m/s", vel);
    return false;

  default:
    setVelCmd(0, 0);
    return true;
  }
}