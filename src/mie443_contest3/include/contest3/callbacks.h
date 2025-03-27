#pragma once

#include "contest3/contest3.h"

class StateVars {
public:
  // Odometer variables
  float posX; // X position relative to start [m]
  float posY; // Y position relative to start [m]
  float yaw;  // Yaw angle [deg]

  StateVars();
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
};
