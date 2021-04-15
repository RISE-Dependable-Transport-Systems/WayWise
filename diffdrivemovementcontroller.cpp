#include "diffdrivemovementcontroller.h"
#include <QDebug>

DiffDriveMovementController::DiffDriveMovementController(QSharedPointer<DiffDriveVehicleState> vehicleState)
{
    mVehicleState = vehicleState;
}

void DiffDriveMovementController::setDesiredSteering(double desiredSteering)
{
    MovementController::setDesiredSteering(desiredSteering);
    // TODO: actual communication with motor controller
    mVehicleState->setSpeedLeft(getDesiredSpeed() + getDesiredSpeed() * desiredSteering);
    mVehicleState->setSpeedRight(getDesiredSpeed() - getDesiredSpeed() * desiredSteering);
}

void DiffDriveMovementController::setDesiredSpeed(double desiredSpeed)
{
    MovementController::setDesiredSpeed(desiredSpeed);
    // TODO: actual communication with motor controller
    mVehicleState->setSpeedLeft(desiredSpeed + desiredSpeed * getDesiredSteering());
    mVehicleState->setSpeedRight(desiredSpeed - desiredSpeed * getDesiredSteering());

}

