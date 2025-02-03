/*
 *     Copyright 2024 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Abstract interface for receiving angle values e.g., magnetic rotary position sensor 
 */

#ifndef ANGLESENSORUPDATER_H
#define ANGLESENSORUPDATER_H

#include <QObject>
#include <QSharedPointer>
#include "vehicles/vehiclestate.h"

class AngleSensorUpdater : public QObject
{
    Q_OBJECT
public:
    AngleSensorUpdater(QSharedPointer<VehicleState> vehicleState);
    QSharedPointer<VehicleState> getVehicleState() const;
    virtual bool setUpdateIntervall(int intervall_ms) = 0;
    virtual bool isConnected() = 0;


signals:
    void updatedAngleSensor(QSharedPointer<VehicleState> vehicleState);

private:
    QSharedPointer<VehicleState> mVehicleState; // vehicle which's angle sensor is periodically updated

};

#endif // ANGLESENSORUPDATER_H
