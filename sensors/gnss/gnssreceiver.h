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


enum class RECEIVER_VARIANT {UNKNOWN, WAYWISE_SIMULATED, EXT_SIMULATED, UBLX_ZED_F9P, UBLX_ZED_F9R};

enum class RECEIVER_STATE
{
    UNKNOWN,
    DISCONNECTED,
    CONNECTED,
    BACKUP_NOT_FOUND,
    BACKUP_RESTORED,
    CONFIGURED,
    CALIBRATING,
    READY,
    BACKUP_ONGOING,
    BACKUP_CREATED
};

enum class GNSS_FIX_TYPE {NO_FIX, DEAD_RECKONING_ONLY, FIX_2D, FIX_3D, FIX_GNSS_DR_COMBINED, FIX_TIME_ONLY};

class GNSSReceiver : public QObject
{
    Q_OBJECT
public:
    GNSSReceiver(QSharedPointer<VehicleState> vehicleState);

    void setReceiverVariant(RECEIVER_VARIANT receiverVariant) { mReceiverVariant = receiverVariant; }
    RECEIVER_VARIANT getReceiverVariant() { return mReceiverVariant; }
    virtual llh_t getEnuRef() const { return mEnuReference; }
    virtual void setEnuRef(llh_t enuRef);
    virtual void setChipOrientationOffset(double roll_deg, double pitch_deg, double yaw_deg) { mAChipOrientationOffset = {roll_deg, pitch_deg, yaw_deg}; }
    virtual void setAntennaToChipOffset(double xOffset, double yOffset, double zOffset) { mAntennaToChipOffset = {xOffset, yOffset, zOffset}; }
    virtual void setChipToRearAxleOffset(double xOffset, double yOffset, double zOffset) { mChipToRearAxleOffset = {xOffset, yOffset, zOffset}; }
    RECEIVER_STATE getReceiverState() { return mReceiverState; }
    void setReceiverState(RECEIVER_STATE state) { mReceiverState = state; }
    virtual void aboutToShutdown() {};
    virtual void readVehicleSpeedForPositionFusion() {};

signals:
    void updatedEnuReference(llh_t mEnuReference);

protected:
    virtual void shutdownGNSSReceiver() {};

    RECEIVER_VARIANT mReceiverVariant = RECEIVER_VARIANT::UNKNOWN;
    RECEIVER_STATE mReceiverState = RECEIVER_STATE::UNKNOWN;
    GNSS_FIX_TYPE mFixType = GNSS_FIX_TYPE::NO_FIX;

    llh_t mEnuReference;
    bool mEnuReferenceSet = false;
    QSharedPointer<VehicleState> mVehicleState;
    struct {double rollOffset_deg, pitchOffset_deg, yawOffset_deg;} mAChipOrientationOffset; // in degrees
    xyz_t mAntennaToChipOffset = {0.0, 0.0, 0.0}; // in meters
    xyz_t mChipToRearAxleOffset = {0.0, 0.0, 0.0}; // in meters
};

#endif // GNSSRECEIVER_H
