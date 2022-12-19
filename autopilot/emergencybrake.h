#ifndef EMERGENCYBRAKE_H
#define EMERGENCYBRAKE_H

#include <QObject>
#include <QTimer>
#include "core/pospoint.h"

struct EmergencyBrakeState {
    bool cameraBrake = false;
    bool lidarBrake = false;
    bool radarBrake = false;
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
    void camera(const PosPoint &detectedObject);

private:
    double objectBrakeDistance = 10; // [m] brake when detected object comes closer than
    EmergencyBrakeState mCurrentState;
    unsigned mUpdateStatePeriod_ms = 50;
    QTimer mUpdateStateTimer;
    void decissionLogic();
};

#endif // EMERGENCYBRAKE_H
