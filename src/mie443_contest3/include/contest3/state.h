#pragma once

#include "contest3/contest3.h"

enum State { START, END };

class RobotState {
private:
  State currState;

public:
  void updateState(float secondsElapsed);
};
