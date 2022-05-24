/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Abstract class to communicate with a vehicle using a remote VehicleConnection
 */

#ifndef VEHICLECONNECTION_H
#define VEHICLECONNECTION_H

#include <QObject>
#include <QDebug>
#include "core/coordinatetransforms.h"
#include "vehicles/vehiclestate.h"
#include "sensors/camera/gimbal.h"

class VehicleConnection : public QObject
{
    Q_OBJECT
public:
    virtual void requestGotoENU(const xyz_t &xyz, bool changeAutopilotMode = false) = 0;
    virtual void requestVelocityAndYaw(const xyz_t &velocityENU, const double &yawDeg) = 0;
    virtual void setActuatorOutput(int index, float value) {qDebug() << "Warning: VehicleConnection::setActuatorOutput() not implemented";}; // TODO: pretty PX4-specific

    QSharedPointer<VehicleState> getVehicleState() const {return mVehicleState;};
    QSharedPointer<Gimbal> getGimbal() const {return mGimbal;};
    bool hasGimbal() const {return !mGimbal.isNull();};

signals:

protected:
    QSharedPointer<VehicleState> mVehicleState;
    QSharedPointer<Gimbal> mGimbal = nullptr;

};

#endif // VEHICLECONNECTION_H
