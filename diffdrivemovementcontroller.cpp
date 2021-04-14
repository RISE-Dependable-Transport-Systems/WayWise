#include "diffdrivemovementcontroller.h"
#include <QDebug>

DiffDriveMovementController::DiffDriveMovementController(QSharedPointer<DiffDriveVehicleState> vehicleState)
{
    mVehicleState = vehicleState;
}

void DiffDriveMovementController::setDesiredSteering(double desiredSteering)
{
    MovementController::setDesiredSteering(desiredSteering);
    mVehicleState->setSpeedLeft(getDesiredSpeed() + getDesiredSpeed() * desiredSteering);
    mVehicleState->setSpeedRight(getDesiredSpeed() - getDesiredSpeed() * desiredSteering);
}

void DiffDriveMovementController::setDesiredSpeed(double desiredSpeed)
{
    MovementController::setDesiredSpeed(desiredSpeed);
    mVehicleState->setSpeedLeft(desiredSpeed + desiredSpeed * getDesiredSteering());
    mVehicleState->setSpeedRight(desiredSpeed - desiredSpeed * getDesiredSteering());

}

