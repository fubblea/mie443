#pragma once

#include "contest3/contest3.h"
#include "geometry_msgs/Twist.h"
#include "kobuki_msgs/BumperEvent.h"
#include "sound_play/sound_play.h"
#include "visualization_msgs/Marker.h"

enum State {
  START,
  FOLLOW_AHEAD, // Happy
  FOLLOW_BACK,  // Disgust
  IM_HIT,       // Angry
  LOST,         // Sad
  END,
};

enum BumperHit { LEFT, CENTER, RIGHT, NOTHING };

class RobotState {
protected:
  State currState = State::START;
  geometry_msgs::Twist velCmd;
  BumperHit bumperHit;

  sound_play::SoundClient &sc;

  geometry_msgs::Twist follow_cmd;
  visualization_msgs::Marker follow_marker;
  uint8_t bumper[NUM_BUMPERS];
  int world_state = 0;

public:
  // Constructors
  RobotState(sound_play::SoundClient &sc) : sc(sc) {};

  // Setters
  void setState(State newState) { this->currState = newState; }
  void setVelCmd(geometry_msgs::Twist velCmd) { this->velCmd = velCmd; }
  void setVelCmd(double linear, double angular) {
    this->velCmd.angular.z = angular;
    this->velCmd.linear.x = linear;
  }

  // Getters
  geometry_msgs::Twist getVelCmd() { return this->velCmd; };

  // State machine
  void updateState(float secondsElapsed, bool contestMode);

  // Callbacks
  void followerCB(const geometry_msgs::Twist::ConstPtr &msg);
  void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr &msg);
  void followerMarkerCB(const visualization_msgs::Marker::ConstPtr &msg);

  bool backAway();
  BumperHit checkBumper();
};
