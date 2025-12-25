#include "Rover.h"

void ModeHold::update()
{
    float throttle = 0.0f;

    // hold position - stop motors and center steering
    g2.motors.set_throttle(throttle);
    g2.motors.set_steering(0.0f);
}
