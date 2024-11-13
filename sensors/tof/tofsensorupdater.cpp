/*
 *     Copyright 2022 RISE Sweden
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "tofsensorupdater.h"

ToFSensorUpdater::ToFSensorUpdater(QSharedPointer<VehicleState> vehicleState)
{
    mVehicleState = vehicleState;
}

QSharedPointer<VehicleState> ToFSensorUpdater::getVehicleState() const
{
    return mVehicleState;
}
