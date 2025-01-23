#include "contest1.h"
#include "state.h"

void robotState::update()
{
    if (currState == State::START) {
        setVelCmd(0, 0);
        setState(State::SPIN);

    } else if (currState == State::SPIN) {
        setVelCmd(5, 0);
    }
}

Vel::Vel(float angular, float linear)
{
    angular = angular;
    linear = linear;
}
