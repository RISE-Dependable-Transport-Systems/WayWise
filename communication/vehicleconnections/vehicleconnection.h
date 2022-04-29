/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Abstract class to communicate with a vehicle using a remote VehicleConnection
 */

#ifndef VEHICLECONNECTION_H
#define VEHICLECONNECTION_H

#include <QObject>
#include "core/coordinatetransforms.h"
#include "vehicles/vehiclestate.h"

class VehicleConnection : public QObject
{
    Q_OBJECT
public:
    virtual void requestGotoENU(const xyz_t &xyz) = 0;

    QSharedPointer<VehicleState> getVehicleState() const {return mVehicleState;};

signals:

protected:
    QSharedPointer<VehicleState> mVehicleState;

private:

};

#endif // VEHICLECONNECTION_H
