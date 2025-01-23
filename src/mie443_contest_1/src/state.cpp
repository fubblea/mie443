#include "contest1.h"
#include "state.h"

void robotState::step()
{
    if (currState == START) {
        setState(State::SPIN);
    }
}

Vel::Vel(float angular, float linear)
{
    angular = angular;
    linear = linear;
}
