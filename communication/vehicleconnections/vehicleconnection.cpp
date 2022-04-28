#include "vehicleconnection.h"

VehicleConnection::VehicleConnection(QObject *parent) : QObject(parent)
{

}

QSharedPointer<VehicleState> VehicleConnection::getVehicleState() const
{
    return mVehicleState;
}
