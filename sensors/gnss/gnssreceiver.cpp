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

void GNSSReceiver::setIMUOrientationOffset(double roll_deg, double pitch_deg, double yaw_deg)
{
    mIMUOrientationOffset = {roll_deg, pitch_deg, yaw_deg};
}

void GNSSReceiver::setGNSSPositionOffset(double xOffset, double yOffset)
{
    mGNSSPositionOffset = {xOffset, yOffset, 0.0};
}
