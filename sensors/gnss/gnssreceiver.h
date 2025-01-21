/*
 *     Copyright 2024 Ramana Avula      ramana.reddy.avula@ri.se
 *               2024 Marvin Damschen   marvin.damschen@ri.se
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
    virtual void setIMUOrientationOffset(double roll_deg, double pitch_deg, double yaw_deg);
    virtual void setGNSSPositionOffset(double xOffset, double yOffset);

signals:
    void updatedEnuReference(llh_t mEnuReference);

protected:
    llh_t mEnuReference;
    bool mEnuReferenceSet = false;
    QSharedPointer<VehicleState> mVehicleState;
    struct {double rollOffset_deg, pitchOffset_deg, yawOffset_deg;} mIMUOrientationOffset;
    xyz_t mGNSSPositionOffset = {0.0, 0.0, 0.0};
};

#endif // GNSSRECEIVER_H
