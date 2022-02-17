#include "imuorientationupdater.h"

IMUOrientationUpdater::IMUOrientationUpdater(QSharedPointer<VehicleState> vehicleState)
{
    mVehicleState = vehicleState;
}

QSharedPointer<VehicleState> IMUOrientationUpdater::getVehicleState() const
{
    return mVehicleState;
}
