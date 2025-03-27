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
    mEnuReference = {57.71495867, 12.89134921, 0}; // AztaZero {57.7810, 12.7692, 0}, Klätterlabbet {57.6876, 11.9807, 0}, RISE RTK base station {57.71495867, 12.89134921, 0}
}

void GNSSReceiver::setEnuRef(llh_t enuRef)
{
    mEnuReference = enuRef;
    mEnuReferenceSet = true;
    emit updatedEnuReference(mEnuReference);
}
