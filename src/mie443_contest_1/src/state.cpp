#include "state.h"
#include "callbacks.h"
#include "contest1.h"
#include "ros/console.h"
#include <cmath>
#include <cstdlib>
#include <limits>

Vel::Vel(float angular, float linear) {
  angular = angular;
  linear = linear;
}

// ===================STATE MACHINE =============================

void robotState::update(float secondsElapsed) {
  switch (currState) {

  // Program start
  case State::START:
    ROS_INFO("Okayyyyy let's go");
    setVelCmd(0, 0);

    if (checkBumper() != BumperHit::NOTHING) {
      setState(State::IM_HIT);
    } else {
      setState(State::SPIN);
    }

    break;

  // Check left side and record wall distance
  case State::CHECK_LEFT:
    ROS_INFO("Checking left side");

    // Check the left side
    if (doTurn(90, stateHist.back().yaw, false)) {
      // Record the distance in front
      std::get<0>(stateVars.sideSpace) = stateVars.wallDist;
      ROS_INFO("Left side wallDist is: %f", std::get<0>(stateVars.sideSpace));

      // Check if the side has been visited before
      std::get<0>(stateVars.sideVisited) = checkVisit(VISITED_BOX);
      ROS_INFO("Has left side been visited?: %i",
               std::get<0>(stateVars.sideVisited));

      setState(State::CHECK_RIGHT);
    }
    break;

  // Check right side and record wall distance
  case State::CHECK_RIGHT:
    ROS_INFO("Checking right side");

    // Check the right side
    if (doTurn(180, stateHist.back().yaw, false)) {
      // Record the distance in front
      std::get<1>(stateVars.sideSpace) = stateVars.wallDist;
      ROS_INFO("Right side wallDist is: %f", std::get<1>(stateVars.sideSpace));

      // Check if the side has been visited before
      std::get<1>(stateVars.sideVisited) = checkVisit(VISITED_BOX);
      ROS_INFO("Has right side been visited?: %i",
               std::get<1>(stateVars.sideVisited));

      setState(State::REORIENT);
    }
    break;

  // Reorient the bot depending on which side has more space
  case State::REORIENT:
    // Check if right side is good first
    if (std::get<1>(stateVars.sideVisited)) {
      ROS_INFO("Right side hasn't been visited. Good to go");
      setState(State::THINK);
    } else if (std::get<0>(stateVars.sideVisited)) {
      ROS_INFO("Left side hasn't been visited. Flipping around");
      if (doTurn(180, stateHist.back().yaw, false)) {
        setState(State::THINK);
      }
    } else {
      ROS_INFO("Neither side explored. Checking distances");

      if (std::get<0>(stateVars.sideSpace) >=
          std::get<1>(stateVars.sideSpace)) {
        ROS_INFO("Left side has more space. Flipping around");
        if (doTurn(180, stateHist.back().yaw, false)) {
          setState(State::THINK);
        }
      } else {
        ROS_INFO("Right side has more space. Good to go");
        setState(State::THINK);
      }
    }

    break;

  // Spinning around
  case State::SPIN:
    ROS_INFO("You spin me right round baby right round like a record baby "
             "right round right round");
    if (doTurn(MAX_SPIN_ANGLE, stateHist.back().yaw, false)) {
      setState(State::THINK);
    }
    break;

  // Think about what to do
  case State::THINK:
    ROS_INFO("Contemplating life");
    if (checkBumper() == BumperHit::NOTHING) {
      if (secondsElapsed > WALL_FOLLOW_TIME) {
        setState(State::WALL_FOLLOW);
      } else {

        if (stateVars.wallDist > MIN_WALL_DIST) {
          ROS_INFO("Distance to wall is %f m, going FAST", stateVars.wallDist);
          setState(State::IM_SPEED);
        } else {
          ROS_INFO("Distance to wall is %f m, checking around",
                   stateVars.wallDist);
          setState(State::IM_HIT);
        }
      }

    } else {
      ROS_INFO("I am close to wall, time to turn");
      setState(State::IM_HIT);
    }
    break;

  // Speed to the wall
  case State::IM_SPEED:
    ROS_INFO("Speed to the wall");

    if (checkBumper() != BumperHit::NOTHING) {
      setState(State::IM_HIT);
    } else if (moveToWall(0, MAX_LIN_VEL)) {
      setState(State::IM_HIT);
    }
    break;

  case State::IM_HIT:
    ROS_INFO("I was hit at bumper %i. Backing away.",
             stateHist.back().bumperHit);
    if (backAway(0.1)) {
      setState(State::CHECK_LEFT);
    }
    break;

  // Program end
  case State::END:
    setVelCmd(0, 0);
    ROS_INFO("Aight I'm outta here");
    ros::shutdown();
    break;

  // Wall following mode
  case State::WALL_FOLLOW:
    ROS_INFO("Following the wall");
    

   //changing state to check hit first

  if(checkBumper() != BumperHit::NOTHING){
    setState(State::IM_HIT);
    break;
  }
  stateVars.frontWallDist = calcFrontWallDist();
  stateVars.sideWallDist = calcSideWallDist();

  ROS_INFO("Wall follow dists. Front: %f, Side: %f",
            stateVars.frontWallDist, stateVars.sideWallDist);
  // To reduce noise near corners, switching to slower speeds near walls
  if(stateVars.frontWallDist < 0.6){
    ROS_INFO("Wall approaching, gotta go slow");
    setVelCmd(0, SLOW_LIN_VEL); //moves slower when approaching wall
  }
  if (stateVars.sideWallDist < MIN_WALL_DIST){
    ROS_INFO("Tight corner gotta go slow");
    setVelCmd(0, SLOW_LIN_VEL); //move slower when in tight spaces
  }
  if (stateVars.frontWallDist < MIN_WALL_DIST) {
    setVelCmd(MAX_ANG_VEL, 0); //reducing speed in tight distances
  } else {
    float anglecmd = RAD2DEG(calcAngleControlCmd(stateVars.sideWallDist));
    setVelCmd(anglecmd,MAX_LIN_VEL);
  }

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
    if (std::fabs(error) <= (ANGLE_TOL * 2)) {
      turn_speed = MIN_ANG_VEL;
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
  if (targetDist < MIN_WALL_DIST) {
    targetDist = MIN_WALL_DIST;
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

bool robotState::checkVisit(float dist) {
  ROS_INFO("Checking Ahead");
  float boxXUB = stateVars.posX;
  float boxYUB = stateVars.posY + dist / 2;
  float boxXLB = stateVars.posX - dist;
  float boxYLB = stateVars.posY - dist / 2;
  ROS_INFO("The box is defined between: (%f, %f) and (%f, %f)", boxXLB, boxYLB,
           boxXUB, boxYUB);
  for (const auto &pos : stateVars.visitedPos) {
    float recordedX = std::get<0>(pos);
    float recordedY = std::get<1>(pos);
    ROS_INFO("checking against: (%f, %f)", std::get<0>(pos), std::get<1>(pos));
    if (recordedX >= boxXLB && recordedX <= boxYLB && recordedY >= boxYLB &&
        recordedY <= boxYUB) {
      ROS_INFO("Visited");
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

void robotState::updateVisitedPos() {
  // Add the current position to the vec of already visited positions
  std::tuple<float, float> currPos =
      std::make_tuple(stateVars.posX, stateVars.posY);
  int cnt = std::count(stateVars.visitedPos.begin(), stateVars.visitedPos.end(),
                       currPos);

  // Add only if not previously added
  if (cnt == 0) {
    stateVars.visitedPos.push_back(currPos);
    ROS_INFO("Added (%f, %f) to visitedPos", std::get<0>(currPos),
             std::get<1>(currPos));
  }
}

float robotState::calcFrontWallDist() {
  int centerIdx = stateVars.rawScanData.ranges.size() / 2;
  float frontDist = std::numeric_limits<float>::max();

  for (int i = -FRONT_DETECT_RANGE / 2; i <= FRONT_DETECT_RANGE / 2; i++) {
    int idx = centerIdx + i;

    if (idx >= 0 &&
        idx < static_cast<int>(stateVars.rawScanData.ranges.size())) {
      frontDist = std::min(frontDist, stateVars.rawScanData.ranges[idx]);
    } else {
      ROS_ERROR("FRONT_DETECT_RANGE invalid: %i", FRONT_DETECT_RANGE);
    }
  }

  return frontDist;
}

float robotState::calcSideWallDist() {
  float sum = 0;
  int count = 0;

  int startIdx;
  int increment;
  int endIdx;
  // Positive is left side
  if (SIDE_DETECT_RANGE >= 0) {
    startIdx = 0;
    increment = 1;
    endIdx = startIdx + std::abs(SIDE_DETECT_RANGE);
  } else {
    startIdx = static_cast<int>(stateVars.rawScanData.ranges.size());
    increment = -1;
    endIdx = startIdx - std::abs(SIDE_DETECT_RANGE);
  }

  for (int i = startIdx; i <= endIdx; i += increment) {
    float dist = stateVars.rawScanData.ranges[i];

    if (std::isfinite(dist) && dist >= stateVars.rawScanData.range_min &&
        dist <= stateVars.rawScanData.range_max) {
      sum += dist;
      count++;
    }
  }

  if (count == 0) {
    ROS_WARN("No valid laser readings for side. Setting to 0");
    return 0;
  } else {
    float avgDist = sum / count;

    // Calculate the perpendicular component
    return avgDist * sin(DEG2RAD(30));
  }
}

float robotState::calcAngleControlCmd(float sideDist) {
  float error = WALL_FOLLOW_DIST - sideDist;
  float controlCmd = CONTROLLER_KP * error;

  ROS_INFO("Wall follow error: %f. Control command: %f", error, controlCmd);
  return controlCmd;
}
