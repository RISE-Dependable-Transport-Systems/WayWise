#include "movementcontroller.h"

MovementController::MovementController(QSharedPointer<VehicleState> vehicleState)
{
    mVehicleState = vehicleState;
}

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

QSharedPointer<VehicleState> MovementController::getVehicleState() const
{
    return mVehicleState;
}
