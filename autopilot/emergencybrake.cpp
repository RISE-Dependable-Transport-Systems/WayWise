/*
 *     Copyright 2023 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "emergencybrake.h"
#include <QDebug>
#include <QTime>
#include <QStringList>

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
    QString time_depthai = msg.mid(2);  // Skip initial character and colon

    QStringList time_depthai_parts = time_depthai.split(':');

    int hours           = time_depthai_parts[0].toInt();
    int minutes         = time_depthai_parts[1].toInt();
    int seconds         = time_depthai_parts[2].toInt();
    int milliseconds    = time_depthai_parts[3].toInt();

    QTime depthaiTime(hours, minutes, seconds, milliseconds);
    QTime rccarTime = QTime::currentTime();

    // qDebug() << "Time DepthAI:" << depthaiTime;
    // qDebug() << "Time RCCar:" << rccarTime;

    qint64 diff_milliseconds = depthaiTime.msecsTo(rccarTime);

    if(msg.at(0) == 'S') {
        mCurrentState.brakeForDetectedCameraObject = true;
        qDebug() << "Stop. Reaction delay:" << diff_milliseconds << "ms";
    } else if (msg.at(0) == 'G') {
        mCurrentState.brakeForDetectedCameraObject = false;
        qDebug() << "Go. Reaction delay:" << diff_milliseconds << "ms";
    } else {
        mCurrentState.brakeForDetectedCameraObject = true;
        qDebug() << "Unknown message received:" << msg << "Activating emergency break.";
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
