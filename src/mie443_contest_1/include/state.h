#ifndef STATE_H
#define STATE_H

#include "contest1.h"

/*
Current state of the robot including state variables.
*/
class robotState {
private:
  State currState = State::START; // Current state
  Vel velCmd = Vel(0, 0);         // Velocity command

public:
  StateVars stateVars = StateVars(); // Current state variables
  std::vector<StateVars> stateHist;  // State reference point

  // Getters

  /*
  Get the robot state
  */
  State getState() const { return currState; };

  /*
  Get the current angular [deg/s] and linear [m/s] velocity command.
  */
  Vel getVelCmd() const { return velCmd; };

  // Setters

  /*
  Set the robot state.
  */
  void setState(State newState) {
    ROS_INFO("Changing state from %i to %i", currState, newState);

    // Take snapshot of current state to use as reference
    StateVars stateRef = StateVars();
    stateRef.posX = stateVars.posX;
    stateRef.posY = stateVars.posY;
    stateRef.yaw = stateVars.yaw;

    stateRef.wallDist = stateVars.wallDist;
    stateRef.wallAngle = stateVars.wallAngle;

    stateRef.bumper[0] = stateVars.bumper[0];
    stateRef.bumper[1] = stateVars.bumper[1];
    stateRef.bumper[2] = stateVars.bumper[2];

    stateRef.bumperHit = stateVars.bumperHit;

    stateRef.mapPose = stateVars.mapPose;
    stateRef.gridIdx = stateVars.gridIdx;
    stateRef.goal = stateVars.goal;

    stateRef.oldState = currState;

    stateHist.push_back(stateRef);

    // Update the state
    currState = newState;
  }

  /*
  Set the angular [deg/s] and linear [m/s] velocites.
  */
  void setVelCmd(float angular, float linear) {
    velCmd.angular = angular;
    velCmd.linear = linear;

    ROS_INFO("Set velCmd to (%f, %f)", velCmd.angular, velCmd.linear);
  }

  /*
  Updates the state of the robot and performs state-specific actions.

  Runs every loop iteration.
  */
  void update(tf::TransformListener &tfListener);

private:
  /*
  Turns the robot to a target angle with respect to a reference point.

  If `quick` is true, it will find the shortest rotation to the target angle,
  otherwise, it do the entire specified rotation.
  */
  bool doTurn(float relativeTarget, float reference, bool quick = true);

  /*
  Move robot straight at the specified speed until the wall is within
  the target distance
  */
  bool moveToWall(float targetDist, float speed = MAX_LIN_VEL);

  /*
  Update the vector of visited positions with the current position.

  Checks current vector to ensure that there aren't any duplicates
  */
  void updateVisitedPos();

  /*
  Updates the position of the robot with respect to the map frame
  */
  void updateMapPose(tf::TransformListener &tfListener);

  /*
  Converts the map pose to occupancy grid indices
  */
  void updateOccGridIdx();

  /*
  Updates the points for the known map on either side of the robot.

  Based on the current orientation.
  */
  void updateSideKnown(float yawOffset);

  /*
  Checks to see if the robot has been here before.

  Uses a box as a tolerance
  */
  bool checkVisit(float posX, float posY, float tol = 0.5);

  /*
  Determine if a bumper is hit and which bumper is hit
  */
  BumperHit checkBumper();

  /*
  Move robot back from the hit
  */
  bool backAway(float desiredDist);

  /*
  Moves until the bumper is hit
  */
  bool moveTilBumped(float vel = MAX_LIN_VEL);

  /*
  Score the specified side based on how much of the map is know.

  The greater the score, the more is known about this side.

  Takes into account the orientation of the robot.

  Applies an additional penalty for completely unknown cells.
  */
  int scoreSideKnown(bool checkLeft, float yawOffset, int searchWidth,
                     int searchDepth);

  std::tuple<float, float> mapIdxToPos(std::tuple<int, int> gridIdx);
  bool setFrontierGoal(std::vector<std::tuple<float, float>> excludedPoints);
  int idxToRowMajor(std::tuple<int, int> gridIdx);
  std::tuple<int, int> rowMajorToIdx(int rowMajorIdx);
};

#endif // STATE_H
