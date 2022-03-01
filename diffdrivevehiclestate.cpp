#include "diffdrivevehiclestate.h"
#include <QDebug>

DiffDriveVehicleState::DiffDriveVehicleState()
{

}

void DiffDriveVehicleState::setSteering(double steering)
{
    VehicleState::setSteering(steering);

    setSpeedLeft(getSpeed() + (getSpeed() * getSteering()));
    setSpeedRight(getSpeed() - (getSpeed() * getSteering()));
}

void DiffDriveVehicleState::updateOdomPositionAndYaw(double drivenDistance, PosType usePosType)
{
    PosPoint currentPosition = getPosition(usePosType);
    double yaw_rad = currentPosition.getYaw() / (180.0/M_PI);

    // TODO: somewhat ugly, updateOdomPositionAndYaw interface in VehicleState needs to be revised
    static QTime lastTimeCalled = QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc());
    QTime thisTimeCalled = QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc());
    int dt_ms = lastTimeCalled.msecsTo(thisTimeCalled);

    double drivenDistLeft = getSpeedLeft() * dt_ms / 1000.0;
    double drivenDistRight = getSpeedRight() * dt_ms / 1000.0;

    // Differential drive kinematic model, getWidth() should be distance between center of left/right wheel
    if (fabs(getSpeedLeft() - getSpeedRight()) > 1e-6) { // Turning
        double turnRadius = (getWidth() *  (getSpeedLeft() + getSpeedRight())) / (2 * (getSpeedLeft() - getSpeedRight()));
        double yawChange = (drivenDistLeft - drivenDistRight) / getWidth();

        // TODO the following part is generic -> move to parent class
        currentPosition.setX(currentPosition.getX() - turnRadius * (sin(-yaw_rad - yawChange) - sin(-yaw_rad)));
        currentPosition.setY(currentPosition.getY() - turnRadius * (cos(-yaw_rad + yawChange) - cos(-yaw_rad)));

        double nextYaw_rad = yaw_rad + yawChange;
        // normalize Yaw
        while (nextYaw_rad > M_PI)
            nextYaw_rad -= 2.0 * M_PI;
        while (nextYaw_rad < -M_PI)
            nextYaw_rad += 2.0 * M_PI;
        currentPosition.setYaw(nextYaw_rad * 180.0 / M_PI);
    } else { // Driving forward
        currentPosition.setX(currentPosition.getX() + cos(-yaw_rad) * drivenDistance);
        currentPosition.setY(currentPosition.getY() + sin(-yaw_rad) * drivenDistance);
    }

    currentPosition.setTime(thisTimeCalled);
    setPosition(currentPosition);

    lastTimeCalled = thisTimeCalled;
}

double DiffDriveVehicleState::steeringCurvatureToSteering(double steeringCurvature)
{
    return (getWidth() / 2.0) * steeringCurvature;
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

void DiffDriveVehicleState::setSpeed(double speed)
{
    setSpeedLeft(speed + (speed * getSteering()));
    setSpeedRight(speed - (speed * getSteering()));
}
