#include "Rover.h"

void ModeManual::_exit()
{
    // clear lateral when exiting manual mode
    g2.motors.set_lateral(0);
}

void ModeManual::update()
{
    float desired_steering, desired_throttle, desired_lateral;
    get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
    get_pilot_desired_lateral(desired_lateral);

    // apply manual steering expo
    desired_steering = 4500.0 * input_expo(desired_steering / 4500, g2.manual_steering_expo);


    // copy RC scaled inputs to outputs
    g2.motors.set_throttle(desired_throttle);
    g2.motors.set_steering(desired_steering, (g2.manual_options & ManualOptions::SPEED_SCALING));
    g2.motors.set_lateral(desired_lateral);
}
