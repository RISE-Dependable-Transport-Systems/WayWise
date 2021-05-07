#include "carmovementcontroller.h"
#include <QDebug>

CarMovementController::CarMovementController(QSharedPointer<CarState> vehicleState): MovementController(vehicleState)
{

}

void CarMovementController::setDesiredSteering(double desiredSteering)
{
    MovementController::setDesiredSteering(desiredSteering);
    // TODO: actual communication with motor controller
    getVehicleState().dynamicCast<CarState>()->setSteering(desiredSteering);
}

void CarMovementController::setDesiredSpeed(double desiredSpeed)
{
    MovementController::setDesiredSpeed(desiredSpeed);
    // TODO: actual communication with motor controller
    getVehicleState().dynamicCast<CarState>()->setSpeed(desiredSpeed);
}
