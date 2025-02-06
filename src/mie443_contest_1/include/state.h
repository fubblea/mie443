#ifndef STATE_H
#define STATE_H

#include "contest1.h"

/*
Possible robot states.
*/
enum State {
  START,
  SPIN,
  THINK,
  IM_HIT,
  IM_SLOW,
  IM_SPEED,
  CHECK_RIGHT,
  CHECK_LEFT,
  REORIENT,
  END
};

/*
Angular and linear velocity
*/
class Vel {
public:
  float angular = 0.0; // Angular velocity [deg/s]
  float linear = 0.0;  // Linear velocity [m/s]

  // Constructor
  Vel(float angular, float linear);
};

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
  void update();

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
};

#endif // STATE_H