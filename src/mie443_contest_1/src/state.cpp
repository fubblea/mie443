#include "contest1.h"
#include "state.h"

void robotState::step()
{
    if (currState == START) {
        ROS_INFO("Let's get started");
    }
}

Vel::Vel(float angular, float linear)
{
    angular = angular;
    linear = linear;
}
