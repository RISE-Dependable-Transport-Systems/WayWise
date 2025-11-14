/*
 *     Copyright 2024 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Abstract base class for GNSS receivers, supporting both real and simulated GNSS data.
 */

#include "gnssreceiver.h"

GNSSReceiver::GNSSReceiver(QSharedPointer<ObjectState> objectState)
{
    mObjectState = objectState;
}

void GNSSReceiver::setEnuRef(llh_t enuRef)
{
    mEnuReference = enuRef;
    mEnuReferenceSet = true;
    emit updatedEnuReference(mEnuReference);
}
