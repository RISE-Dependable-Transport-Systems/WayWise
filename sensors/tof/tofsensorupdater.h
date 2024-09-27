/*
 *     Copyright 2024 RISE Sweden
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 * 
 * Abstract interface for receiving tof values e.g., magnetic rotary position sensor 
 */

#ifndef TOFSENSORUPDATER_H
#define TOFSENSORUPDATER_H

#include <QObject>
#include <QSharedPointer>
#include "vehicles/vehiclestate.h"

class ToFSensorUpdater : public QObject
{
    Q_OBJECT
public:
    ToFSensorUpdater(QSharedPointer<VehicleState> vehicleState);
    QSharedPointer<VehicleState> getVehicleState() const;
    virtual bool setUpdateIntervall(int intervall_ms) = 0;


signals:
    void updatedtofSensor(QSharedPointer<VehicleState> vehicleState);

private:
    QSharedPointer<VehicleState> mVehicleState; // vehicle which's tof sensor is periodically updated

};

#endif // TOFSENSORUPDATER_H
