#ifndef IMUORIENTATIONUPDATER_H
#define IMUORIENTATIONUPDATER_H

#include <QObject>
#include <QSharedPointer>
#include "vehicles/vehiclestate.h"

class IMUOrientationUpdater : public QObject
{
    Q_OBJECT
public:
    IMUOrientationUpdater(QSharedPointer<VehicleState> vehicleState);
    QSharedPointer<VehicleState> getVehicleState() const;
    virtual bool setUpdateIntervall(int intervall_ms) = 0;


signals:
    void updatedIMUOrientation(QSharedPointer<VehicleState> vehicleState);

private:
    QSharedPointer<VehicleState> mVehicleState; // vehicle which's PosType::IMU is periodically updated

};

#endif // IMUORIENTATIONUPDATER_H
