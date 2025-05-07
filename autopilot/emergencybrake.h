/*
 *     Copyright 2023 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Uses sensor inputs to take emergency brake decision
 */

#ifndef EMERGENCYBRAKE_H
#define EMERGENCYBRAKE_H

#include <QObject>
#include "core/pospoint.h"

struct EmergencyBrakeState {
    bool brakeForDetectedCameraObject = false;
    bool brakeForDetectedLidarObject = false;
    bool brakeForDetectedRadarObject = false;
    double brakeForObjectAtDistance = 10; // [m] brake when detected object comes closer than
    bool emergencyBrakeIsActive = false;
};

class EmergencyBrake : public QObject
{
    Q_OBJECT
public:
    explicit EmergencyBrake(QObject *parent = nullptr);

signals:
    void emergencyBrake();

public slots:
    void deactivateEmergencyBrake();
    void activateEmergencyBrake();
    void brakeForDetectedCameraObject(const QString& msg);

private:
    EmergencyBrakeState mCurrentState;
    void fuseSensorsAndTakeBrakeDecision();
};

#endif // EMERGENCYBRAKE_H
