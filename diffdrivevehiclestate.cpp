#include "diffdrivevehiclestate.h"
#include <QDebug>

DiffDriveVehicleState::DiffDriveVehicleState()
{

}

void DiffDriveVehicleState::simulationStep(double dt_ms)
{
    PosPoint currentPosition = getPosition();
    double drivenDistance = getSpeed() * dt_ms / 1000;
    double drivenDistLeft = getSpeedLeft() * dt_ms / 1000;
    double drivenDistRight = getSpeedRight() * dt_ms / 1000;

    // Differential drive kinematic model, TODO: getWidth() should be center of left/right wheel
    if (fabs(getSpeedLeft() - getSpeedRight()) > 1e-6) { // Turning
        double turnRadius = (getWidth() *  (drivenDistLeft + drivenDistRight)) / (2 * (drivenDistLeft - drivenDistRight));
        double yawChange = (drivenDistLeft - drivenDistRight) / getWidth();

        // TODO the following part is generic -> move to parent class
        currentPosition.setX(currentPosition.getX() - turnRadius * (sin(-currentPosition.getYaw() - yawChange) - sin(-currentPosition.getYaw())));
        currentPosition.setY(currentPosition.getY() - turnRadius * (cos(-currentPosition.getYaw() + yawChange) - cos(-currentPosition.getYaw())));

        double nextYaw = currentPosition.getYaw() + yawChange;
        // normalize Yaw
        while (nextYaw > M_PI)
            nextYaw -= 2.0 * M_PI;
        while (nextYaw < -M_PI)
            nextYaw += 2.0 * M_PI;
        currentPosition.setYaw(nextYaw);
    } else { // Driving forward
        currentPosition.setX(currentPosition.getX() + cos(-currentPosition.getYaw()) * drivenDistance);
        currentPosition.setY(currentPosition.getY() + sin(-currentPosition.getYaw()) * drivenDistance);
    }

    setPosition(currentPosition);
}

double DiffDriveVehicleState::getSpeedLeft() const
{
    return mSpeedLeft;
}

void DiffDriveVehicleState::setSpeedLeft(double speedLeft)
{
    mSpeedLeft = speedLeft;
}

double DiffDriveVehicleState::getSpeedRight() const
{
    return mSpeedRight;
}

void DiffDriveVehicleState::setSpeedRight(double speedRight)
{
    mSpeedRight = speedRight;
}

double DiffDriveVehicleState::getSpeed() const
{
    return (getSpeedLeft() + getSpeedRight()) / 2.0;
}
