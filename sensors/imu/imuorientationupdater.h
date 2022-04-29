/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 * 
 * Abstract interface for receiving orientation data from IMUs
 */

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
