/*
 *     Copyright 2024 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Abstract base class for GNSS receivers, supporting both real and simulated GNSS data.
 */

#include "gnssreceiver.h"

GNSSReceiver::GNSSReceiver(QSharedPointer<VehicleState> vehicleState)
{
    mVehicleState = vehicleState;
}

bool GNSSReceiver::simulationStep(const std::function<bool(QTime, QSharedPointer<VehicleState>)> &perturbationFn)
{
    static PosPoint prevOdomPosition = PosPoint();
    PosPoint odomPosition = mVehicleState->getPosition(PosType::odom);

    double deltaX = odomPosition.getX() - prevOdomPosition.getX();
    double deltaY = odomPosition.getY() - prevOdomPosition.getY();
    double deltaYaw = odomPosition.getYaw() - prevOdomPosition.getYaw();

    PosPoint gnssPosition = mVehicleState->getPosition(PosType::GNSS);
    gnssPosition.setX(gnssPosition.getX() + deltaX);
    gnssPosition.setY(gnssPosition.getY() + deltaY);
    double yawResult = gnssPosition.getYaw() + deltaYaw;

    while (yawResult < -180.0)
        yawResult += 360.0;
    while (yawResult >= 180.0)
        yawResult -= 360.0;

    gnssPosition.setYaw(yawResult);
    gnssPosition.setTime(odomPosition.getTime());
    mVehicleState->setPosition(gnssPosition);
    prevOdomPosition = odomPosition;

    if (perturbationFn) {
        return perturbationFn(odomPosition.getTime(), mVehicleState);
    }

    return true;
}
