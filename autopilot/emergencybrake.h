#ifndef EMERGENCYBRAKE_H
#define EMERGENCYBRAKE_H

#include <QObject>
#include <QTimer>
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
    void brakeForDetectedCameraObject(const PosPoint &detectedObject);

private:
    EmergencyBrakeState mCurrentState;
    void fuseSensorsAndTakeBrakeDecision();
};

#endif // EMERGENCYBRAKE_H
