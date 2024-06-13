/*
 *     Copyright 2023 Rickard Häll   rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "emergencybrake.h"

EmergencyBrake::EmergencyBrake(QObject *parent)
    : QObject{parent}
{

}

void EmergencyBrake::deactivateEmergencyBrake()
{
    mCurrentState.emergencyBrakeIsActive = false;
};

void EmergencyBrake::activateEmergencyBrake()
{
    mCurrentState.emergencyBrakeIsActive = true;
};

void EmergencyBrake::brakeForDetectedCameraObject(const PosPoint &detectedObject)
{
    //When no object is detected, objectDistance is zero
    double objectDistance = sqrt(detectedObject.getX()*detectedObject.getX() + detectedObject.getY()*detectedObject.getY() + detectedObject.getHeight()*detectedObject.getHeight());

    if (objectDistance > 0 && objectDistance < mCurrentState.brakeForObjectAtDistance)
        mCurrentState.brakeForDetectedCameraObject = true;
    else
        mCurrentState.brakeForDetectedCameraObject = false;

    fuseSensorsAndTakeBrakeDecision();
};

void EmergencyBrake::fuseSensorsAndTakeBrakeDecision()
{
    if (mCurrentState.emergencyBrakeIsActive) {
        // ToDo: create decission logic depending on multiple sensor inputs
        if (mCurrentState.brakeForDetectedCameraObject)
            emit emergencyBrake();
    }
}
