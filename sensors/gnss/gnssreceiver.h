/*
 *     Copyright 2024 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Abstract base class for GNSS receivers, supporting both real and simulated GNSS data.
 */

#ifndef GNSSRECEIVER_H
#define GNSSRECEIVER_H

#include <QObject>
#include <QSharedPointer>

#include "core/coordinatetransforms.h"
#include "vehicles/vehiclestate.h"

class GNSSReceiver : public QObject
{
    Q_OBJECT
public:
    GNSSReceiver(QSharedPointer<VehicleState> vehicleState);

    virtual llh_t getEnuRef() const { return mEnuReference; }
    virtual void setEnuRef(llh_t enuRef);
    virtual void setChipOrientationOffset(double roll_deg, double pitch_deg, double yaw_deg) { mAChipOrientationOffset = {roll_deg, pitch_deg, yaw_deg}; }
    virtual void setAntennaToChipOffset(double xOffset, double yOffset, double zOffset) { mAntennaToChipOffset = {xOffset, yOffset, zOffset}; }
    virtual void setChipToRearAxleOffset(double xOffset, double yOffset, double zOffset) { mChipToRearAxleOffset = {xOffset, yOffset, zOffset}; }

signals:
    void updatedEnuReference(llh_t mEnuReference);

protected:
    llh_t mEnuReference;
    bool mEnuReferenceSet = false;
    QSharedPointer<VehicleState> mVehicleState;
    struct {double rollOffset_deg, pitchOffset_deg, yawOffset_deg;} mAChipOrientationOffset; // in degrees
    xyz_t mAntennaToChipOffset = {0.0, 0.0, 0.0}; // in meters
    xyz_t mChipToRearAxleOffset = {0.0, 0.0, 0.0}; // in meters
};

#endif // GNSSRECEIVER_H
