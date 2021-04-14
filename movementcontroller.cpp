#include "movementcontroller.h"

double MovementController::getDesiredSteering() const
{
    return mDesiredSteering;
}

void MovementController::setDesiredSteering(double desiredSteering)
{
    mDesiredSteering = desiredSteering;
}

double MovementController::getDesiredSpeed() const
{
    return mDesiredSpeed;
}

void MovementController::setDesiredSpeed(double desiredSpeed)
{
    mDesiredSpeed = desiredSpeed;
}
