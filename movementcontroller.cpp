#include "movementcontroller.h"
#include <QDebug>

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

void MovementController::setDesiredSteeringCurvature(double desiredSteeringAngle)
{
    mDesiredSteeringAngle = desiredSteeringAngle;
}

double MovementController::getDesiredSteeringCurvature() const
{
    return mDesiredSteeringAngle;
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
