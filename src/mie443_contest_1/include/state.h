#ifndef STATE_H
#define STATE_H


enum State {
    START,
    SPIN,
    FIND_WALL,
    END
};

class Vel {
public:
    float angular = 0.0;
    float linear = 0.0;

    // Constructor
    Vel(float angular, float linear);
};

class robotState {
private:
    State currState = State::START;
    Vel velCmd = Vel(0, 0);

public:
    // Getters
    State getState() const { return currState; };
    Vel getVelCmd() const { return velCmd; };

    // Setters
    void setState(State newState) {
        ROS_INFO("Changing state from %i to %i", currState, newState);
        currState = newState;
    }

    void step();
};

#endif // STATE_H