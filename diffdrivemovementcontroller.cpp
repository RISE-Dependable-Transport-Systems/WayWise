#include "diffdrivemovementcontroller.h"
#include <QDebug>

DiffDriveMovementController::DiffDriveMovementController(QSharedPointer<DiffDriveVehicleState> vehicleState): MovementController(vehicleState)
{
    mDiffDriveVehicleState = getVehicleState().dynamicCast<DiffDriveVehicleState>();
}

void DiffDriveMovementController::setDesiredSteering(double desiredSteering)
{
    MovementController::setDesiredSteering(desiredSteering);
    // TODO: actual communication with motor controller
    mDiffDriveVehicleState->setSpeedLeft(getDesiredSpeed() + (getDesiredSpeed() * desiredSteering));
    mDiffDriveVehicleState->setSpeedRight(getDesiredSpeed() - (getDesiredSpeed() * desiredSteering));
}

void DiffDriveMovementController::setDesiredSpeed(double desiredSpeed)
{
    MovementController::setDesiredSpeed(desiredSpeed);
    // TODO: actual communication with motor controller
    mDiffDriveVehicleState->setSpeedLeft(desiredSpeed + desiredSpeed * getDesiredSteering());
    mDiffDriveVehicleState->setSpeedRight(desiredSpeed - desiredSpeed * getDesiredSteering());

}
