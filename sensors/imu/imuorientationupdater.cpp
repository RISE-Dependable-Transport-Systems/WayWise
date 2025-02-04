/*
 *     Copyright 2022 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "imuorientationupdater.h"

IMUOrientationUpdater::IMUOrientationUpdater(QSharedPointer<VehicleState> vehicleState)
{
    mVehicleState = vehicleState;
}

QSharedPointer<VehicleState> IMUOrientationUpdater::getVehicleState() const
{
    return mVehicleState;
}
