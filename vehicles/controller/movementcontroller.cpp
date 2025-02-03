/*
 *     Copyright 2021 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
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

void MovementController::setDesiredSteeringCurvature(double desiredSteeringCurvature)
{
    setDesiredSteering(mVehicleState->steeringCurvatureToSteering(desiredSteeringCurvature));
}

double MovementController::getDesiredSpeed() const
{
    return mDesiredSpeed;
}

void MovementController::setDesiredSpeed(double desiredSpeed)
{
    mDesiredSpeed = desiredSpeed;
}

void MovementController::setDesiredAttributes(quint32 desiredAttributes)
{
    mDesiredAttributes = desiredAttributes;
}

QSharedPointer<VehicleState> MovementController::getVehicleState() const
{
    return mVehicleState;
}
