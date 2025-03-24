/*
 *     Copyright 2023 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "emergencybrake.h"
#include <QDebug>

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

void EmergencyBrake::brakeForDetectedCameraObject(const QString& msg)
{
    if(msg == "1")
        mCurrentState.brakeForDetectedCameraObject = false;
    else {
        mCurrentState.brakeForDetectedCameraObject = true;
        qDebug() << "Activating emergency break:" << QDateTime::currentDateTime().time();
    }
    fuseSensorsAndTakeBrakeDecision();
};

void EmergencyBrake::fuseSensorsAndTakeBrakeDecision()
{
    if (mCurrentState.emergencyBrakeIsActive) {
        // ToDo: create decision logic depending on multiple sensor inputs
        if (mCurrentState.brakeForDetectedCameraObject)
            emit emergencyBrake();
    }
}
