#include "emergencybrake.h"

EmergencyBrake::EmergencyBrake(QObject *parent)
    : QObject{parent}
{
    connect(&mUpdateStateTimer, &QTimer::timeout, this, &EmergencyBrake::decissionLogic);
}

void EmergencyBrake::deactivateEmergencyBrake()
{
    mUpdateStateTimer.stop();
};

void EmergencyBrake::activateEmergencyBrake()
{
    mUpdateStateTimer.start(mUpdateStatePeriod_ms);
};

void EmergencyBrake::camera(const PosPoint &point)
{
    //When no object is detected, objectDistance is zero
    double objectDistance = sqrt(point.getX()*point.getX() + point.getY()*point.getY() + point.getHeight()*point.getHeight());

    if (objectDistance > 0 && objectDistance < objectBrakeDistance)
        mCurrentState.cameraBrake = true;
    else
        mCurrentState.cameraBrake = false;
};

void EmergencyBrake::decissionLogic()
{
    // ToDo: create decission logic depending on multiple sensor inputs
    if (mCurrentState.cameraBrake)
        emit emergencyBrake();
}
