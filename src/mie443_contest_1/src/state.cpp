#include "state.h"
#include "contest1.h"

// =================== STATE MACHINE START =============================

void robotState::update(tf::TransformListener &tfListener) {
  float distToGoal;
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

  // Check left side and record wall distance
  case State::CHECK_LEFT:
    ROS_INFO("Checking left side");
    // Check the left side
    if (doTurn(90, stateHist.back().yaw, false)) {
      // Record the distance in front
      std::get<0>(stateVars.sideSpace) = stateVars.wallDist;

      ROS_INFO("Left side wallDist is: %f", std::get<0>(stateVars.sideSpace));
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
      setState(State::DO_MATH);
    }
    break;

  // Calculate the left and right scores
  case State::DO_MATH:
    // First check how known either side is and score it
    updateSideKnown(-90); // This has a loop, won't exit until done

    std::get<0>(stateVars.sideScore) =
        SPACE_WEIGHT * std::get<0>(stateVars.sideSpace) +
        KNOWN_WEIGHT * std::get<0>(stateVars.sideKnown);
    std::get<1>(stateVars.sideScore) =
        SPACE_WEIGHT * std::get<1>(stateVars.sideSpace) +
        KNOWN_WEIGHT * std::get<1>(stateVars.sideKnown);

    ROS_INFO("Side known: %i left, %i right", std::get<0>(stateVars.sideKnown),
             std::get<1>(stateVars.sideKnown));

    ROS_INFO("Side score: %f left, %f right", std::get<0>(stateVars.sideScore),
             std::get<1>(stateVars.sideScore));

    setState(State::REORIENT_SPACE);
    break;

  case State::REORIENT_SPACE:
    ROS_INFO("Reorienting towards space");

    if (std::get<0>(stateVars.sideScore) >= std::get<1>(stateVars.sideScore)) {
      ROS_INFO("Left side has more space, flipping around");
      if (doTurn(180, stateHist.back().yaw, true)) {
        setState(IM_CHECKING);
      }
    } else {
      ROS_INFO("Right side has more space, good to go");
      setState(IM_CHECKING);
    }
    break;

  // Reorient the bot towards the goal
  case State::REORIENT_GOAL:
    ROS_INFO("Reorienting towards frontier goal: (%f, %f)", stateVars.goal.posX,
             stateVars.goal.posY);
    ROS_INFO("Ref point: (%f, %f)", stateHist.back().mapPose.posX,
             stateHist.back().mapPose.posY);

    // Find vector angle from current pos to goal
    if (doTurn(
            RAD2DEG(atan2(stateVars.goal.posY - stateHist.back().mapPose.posY,
                          stateVars.goal.posX - stateHist.back().mapPose.posX) -
                    DEG2RAD(stateHist.back().mapPose.yaw)),
            stateHist.back().mapPose.yaw, true)) {

      distToGoal =
          sqrt(powf((stateVars.goal.posX - stateHist.back().mapPose.posX), 2) +
               powf((stateVars.goal.posY - stateHist.back().mapPose.posY), 2));

      ROS_INFO("Goal dist: %f", distToGoal);
      ROS_INFO("Wall dist: %f", stateVars.wallDist);

      if ((stateVars.wallDist < distToGoal) &&
          (stateVars.excludeAttempts < MAX_EXCLUSIONS) &&
          distToGoal > MIN_WALL_DIST) {
        ROS_WARN("Excluding frontier point: (%f, %f)", stateVars.goal.posX,
                 stateVars.goal.posY);
        stateVars.excludedPoints.push_back(
            std::make_tuple(stateVars.goal.posX, stateVars.goal.posY));
        stateVars.excludeAttempts++;

        setState(State::THINK);
      } else if (stateVars.wallDist > distToGoal &&
                 stateVars.excludeAttempts < MAX_EXCLUSIONS) {
        ROS_INFO("Wall is not in the way. Let's ago");
        stateVars.excludeAttempts = 0;
        setState(State::IM_SPEED);
      } else {
        ROS_WARN("Can't find a good goal. Making space");
        stateVars.excludeAttempts = 0;
        setState(State::CHECK_LEFT);
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

      if (setFrontierGoal(stateVars.excludedPoints)) {
        setState(State::REORIENT_GOAL);
      }
    }

    break;

  // Speed to the wall
  case State::IM_SPEED:
    ROS_INFO("Speed to the wall");
    if (moveToWall(MIN_WALL_DIST, MAX_LIN_VEL)) {
      setState(State::IM_HIT);
    }
    break;

  // Slowly creep to wall
  case State::IM_CHECKING:
    ROS_INFO("Speeding to wall to make space");
    if (moveToWall(MIN_WALL_DIST, MAX_LIN_VEL)) {
      setState(State::IM_HIT);
    }
    break;

  case State::IM_HIT:
    ROS_INFO("I was hit at bumper %i. Backing away.",
             stateHist.back().bumperHit);
    if (backAway(0.1)) {
      if (stateHist.back().oldState == State::IM_SPEED) {
        setState(State::THINK);
      } else {
        setState(State::CHECK_LEFT);
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
  updateMapPose(tfListener);
}

// =================== STATE MACHINE END =============================

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

int robotState::scoreSideKnown(bool checkLeft, float yawOffset, int searchWidth,
                               int searchDepth) {
  int startX = std::get<0>(stateVars.gridIdx);
  int startY = std::get<1>(stateVars.gridIdx);
  int yaw = stateVars.mapPose.yaw + yawOffset;

  int totalScore = 0;
  int cellCount = 0;

  // Direction base on left or right
  int dir = checkLeft ? -1 : 1;

  // Costmap index step based on direction of the robot
  int dx = round(dir * cos(yaw));
  int dy = round(-dir * sin(yaw));

  bool earlyBreak = false;
  for (int i = 0; i < searchDepth; i++) {
    for (int j = 0; j < searchWidth; j++) {
      int x = startX + i * dx + j * dy;
      int y = startY + i * dy + j * dx;

      // Check to make sure point is within map bounds
      if (x >= 0 && x < stateVars.map.info.width && y >= 0 &&
          y < stateVars.map.info.height) {
        int idx = y * stateVars.map.info.width + x;

        if (stateVars.map.data[idx] < 50) {
          // totalScore += stateVars.map.data[idx] * UNKNOWN_WEIGHT;
          totalScore++;
          cellCount++;
        } else {
          // totalScore += stateVars.map.data[idx];
          totalScore += 0;
          cellCount++;
        }
      } else {
        ROS_WARN("Value not in map range, early break");
        earlyBreak = true;
        break;
      }
    }

    if (earlyBreak) {
      break;
    }
  }

  return totalScore / cellCount; // Normalize by cell count
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

void robotState::updateSideKnown(float yawOffset) {
  int searchWidth = MAP_SEARCH_WIDTH;
  int searchDepth = MAP_SEARCH_DEPTH;

  if (MAP_SEARCH_WIDTH > stateVars.map.info.width) {
    ROS_WARN("MAP_SEARCH_WIDTH is more than actual width: %i. Set to %i",
             MAP_SEARCH_WIDTH, stateVars.map.info.width);
    searchWidth = stateVars.map.info.width / 2;
  }

  if (MAP_SEARCH_DEPTH > stateVars.map.info.height) {
    ROS_WARN("MAP_SEARCH_DEPTH is more than actual height: %i. Set to %i",
             MAP_SEARCH_DEPTH, stateVars.map.info.height);
    searchDepth = stateVars.map.info.height / 2;
  }

  std ::get<0>(stateVars.sideKnown) =
      scoreSideKnown(true, yawOffset, searchWidth, searchDepth);
  std::get<1>(stateVars.sideKnown) =
      scoreSideKnown(false, yawOffset, searchWidth, searchDepth);
}

void robotState::updateOccGridIdx() {
  float resolution = stateVars.map.info.resolution;

  float originX = stateVars.map.info.origin.position.x;
  float originY = stateVars.map.info.origin.position.y;

  std::get<0>(stateVars.gridIdx) =
      (stateVars.mapPose.posX - originX) / resolution;
  std::get<1>(stateVars.gridIdx) =
      (stateVars.mapPose.posY - originY) / resolution;
};

void robotState::updateMapPose(tf::TransformListener &tfListener) {
  tf::StampedTransform transform;

  try {
    tfListener.lookupTransform("map", "base_link", ros::Time(0), transform);

    // Convert robot position to map coordinates
    stateVars.mapPose.posX = transform.getOrigin().x();
    stateVars.mapPose.posY = transform.getOrigin().y();
    stateVars.mapPose.yaw = RAD2DEG(tf::getYaw(transform.getRotation()));

    // Update the occupancy grid index
    updateOccGridIdx();

  } catch (tf::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }
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

std::tuple<float, float> robotState::mapIdxToPos(std::tuple<int, int> gridIdx) {
  float resolution = stateVars.map.info.resolution;

  float originX = stateVars.map.info.origin.position.x;
  float originY = stateVars.map.info.origin.position.y;

  float posX = (std::get<0>(gridIdx) * resolution) + originX;
  float posY = (std::get<1>(gridIdx) * resolution) + originY;

  return std::make_tuple(posX, posY);
}

bool checkInVec(std::tuple<float, float> testPoint,
                std::vector<std::tuple<float, float>> checkPoints) {
  int cnt = std::count(checkPoints.begin(), checkPoints.end(), testPoint);

  // This means it's not in the vector
  if (cnt <= 0) {
    return true;
  } else {
    return false;
  }
}

bool robotState::setFrontierGoal(
    std::vector<std::tuple<float, float>> excludedPoints) {
  int searchSize = START_SEARCH_SIZE;
  int searchAttempts = 0;

  while (searchAttempts < MAX_SEARCH_ATTEMPTS) {
    // Find the indices for the search space
    int startX = std::max(0, std::get<0>(stateVars.gridIdx) - searchSize);
    int endX = std::min(static_cast<int>(stateVars.map.info.width),
                        std::get<0>(stateVars.gridIdx) + searchSize);

    int startY = std::max(0, std::get<1>(stateVars.gridIdx) - searchSize);
    int endY = std::min(static_cast<int>(stateVars.map.info.height),
                        std::get<1>(stateVars.gridIdx) + searchSize);

    ROS_INFO("Starting frontier search. X=[%i, %i], Y=[%i, %i]", startX, endX,
             startY, endY);

    for (int x = startX; x <= endX; x++) {
      for (int y = startY; y <= endY; y++) {
        int idx = y * stateVars.map.info.width + x;
        std::tuple<float, float> goal = mapIdxToPos(std::make_tuple(x, y));

        if (stateVars.map.data[idx] == -1 && checkInVec(goal, excludedPoints)) {
          stateVars.goal.posX = std::get<0>(goal);
          stateVars.goal.posY = std::get<1>(goal);
          ROS_INFO("Found frontier goal: (%f, %f). Idx: (%i, %i)",
                   stateVars.goal.posX, stateVars.goal.posY, x, y);
          return true;
        }
      }
    }

    ROS_WARN("Could not find a frontier point. Increasing search size");
    searchSize += START_SEARCH_SIZE;
    searchAttempts++;
  }

  ROS_ERROR("Could not find frontier goal.");
  return false;
}