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

    virtual void setIMUOrientationOffset(double roll_deg, double pitch_deg, double yaw_deg);
    virtual void setGNSSPositionOffset(double xOffset, double yOffset);

protected:
    QSharedPointer<VehicleState> mVehicleState;
    struct {double rollOffset_deg, pitchOffset_deg, yawOffset_deg;} mIMUOrientationOffset;
    xyz_t mGNSSPositionOffset = {0.0, 0.0, 0.0};
};

#endif // GNSSRECEIVER_H
