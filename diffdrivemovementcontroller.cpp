#include "diffdrivemovementcontroller.h"
#include <QDebug>

DiffDriveMovementController::DiffDriveMovementController(QSharedPointer<DiffDriveVehicleState> vehicleState): MovementController(vehicleState)
{

}

void DiffDriveMovementController::setDesiredSteering(double desiredSteering)
{
    MovementController::setDesiredSteering(desiredSteering);
    // TODO: actual communication with motor controller
    getVehicleState().dynamicCast<DiffDriveVehicleState>()->setSpeedLeft(getDesiredSpeed() + (getDesiredSpeed() * desiredSteering));
    getVehicleState().dynamicCast<DiffDriveVehicleState>()->setSpeedRight(getDesiredSpeed() - (getDesiredSpeed() * desiredSteering));
}

void DiffDriveMovementController::setDesiredSpeed(double desiredSpeed)
{
    MovementController::setDesiredSpeed(desiredSpeed);
    // TODO: actual communication with motor controller
    getVehicleState().dynamicCast<DiffDriveVehicleState>()->setSpeedLeft(desiredSpeed + desiredSpeed * getDesiredSteering());
    getVehicleState().dynamicCast<DiffDriveVehicleState>()->setSpeedRight(desiredSpeed - desiredSpeed * getDesiredSteering());

}

