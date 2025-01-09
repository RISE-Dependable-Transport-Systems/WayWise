/*
 *     Copyright 2024 Ramana Avula      ramana.reddy.avula@ri.se
 *               2024 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Abstract base class for GNSS receivers, supporting both real and simulated GNSS data.
 */

#include "gnssreceiver.h"

GNSSReceiver::GNSSReceiver(QSharedPointer<VehicleState> vehicleState)
{
    mVehicleState = vehicleState;
    mEnuReference = {57.71495867, 12.89134921, 0}; // AztaZero {57.7810, 12.7692, 0}, Klätterlabbet {57.6876, 11.9807, 0}, RISE RTK base station {57.71495867, 12.89134921, 0}
}

void GNSSReceiver::setEnuRef(llh_t enuRef)
{
    mEnuReference = enuRef;
    mEnuReferenceSet = true;
    emit updatedEnuReference(mEnuReference);
}

void GNSSReceiver::setIMUOrientationOffset(double roll_deg, double pitch_deg, double yaw_deg)
{
    mIMUOrientationOffset = {roll_deg, pitch_deg, yaw_deg};
}

void GNSSReceiver::setGNSSPositionOffset(double xOffset, double yOffset)
{
    mGNSSPositionOffset = {xOffset, yOffset, 0.0};
}
