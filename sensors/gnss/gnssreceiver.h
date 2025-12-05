/*
 *     Copyright 2024 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Abstract base class for GNSS receivers, supporting both real and simulated GNSS data.
 */

#ifndef GNSSRECEIVER_H
#define GNSSRECEIVER_H

#include <QDebug>
#include <QLineF>
#include <QObject>
#include <QSharedPointer>
#include <limits>

#include "core/coordinatetransforms.h"
#include "vehicles/vehiclestate.h"


enum class RECEIVER_VARIANT
{
    UNKNOWN,
    WAYWISE_SIMULATED,
    EXTERNAL,
    UBLX_ZED_F9P,
    UBLX_ZED_F9R
};

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

enum class GNSS_FIX_TYPE
{
    NO_FIX,
    DEAD_RECKONING_ONLY_FIX,
    FIX_2D,
    FIX_3D,
    GNSS_DR_COMBINED_FIX,
    TIME_ONLY_FIX
};
struct GnssFixStatus
{
    float horizontalAccuracy = std::numeric_limits<float>::infinity();
    float verticalAccuracy = std::numeric_limits<float>::infinity();
    float headingAccuracy = std::numeric_limits<float>::infinity();
    bool isFused = false;
    GNSS_FIX_TYPE fixType = GNSS_FIX_TYPE::NO_FIX;
    uint8_t lastRtcmCorrectionAge = 0;
    uint8_t numSatellites = 0;
};

class GNSSReceiver : public QObject
{
    Q_OBJECT
public:
    GNSSReceiver(QSharedPointer<ObjectState> objectState);

    void simulationStep(const std::function<GnssFixStatus(QTime, QSharedPointer<ObjectState>)> &simulationFn = nullptr);

    void setReceiverVariant(RECEIVER_VARIANT receiverVariant) { mReceiverVariant = receiverVariant; }
    RECEIVER_VARIANT getReceiverVariant() { return mReceiverVariant; }
    virtual void setChipOrientationOffset(double roll_deg, double pitch_deg, double yaw_deg) { mAChipOrientationOffset = {roll_deg, pitch_deg, yaw_deg}; }
    virtual void setAntennaToChipOffset(double xOffset, double yOffset, double zOffset) { mAntennaToChipOffset = {xOffset, yOffset, zOffset}; }
    virtual void setChipToBaseOffset(double xOffset, double yOffset, double zOffset) { mChipToBaseOffset = {xOffset, yOffset, zOffset}; }
    RECEIVER_STATE getReceiverState() { return mReceiverState; }
    void setReceiverState(RECEIVER_STATE state) { mReceiverState = state; }
    virtual void aboutToShutdown() {};
    virtual void updateGNSSPositionAndYaw(llh_t llh, double heading, bool isFused);

signals:
    void updatedGNSSPositionAndYaw(QSharedPointer<ObjectState> objectState, double distanceMoved, GnssFixStatus gnssFixStatus);

protected:
    virtual void shutdownGNSSReceiver() {};

    // Static variables
    RECEIVER_VARIANT mReceiverVariant = RECEIVER_VARIANT::UNKNOWN;
    struct {double rollOffset_deg, pitchOffset_deg, yawOffset_deg;} mAChipOrientationOffset; // in degrees
    xyz_t mAntennaToChipOffset = {0.0, 0.0, 0.0}; // in meters
    xyz_t mChipToBaseOffset = {0.0, 0.0, 0.0}; // in meters

    // Dynamic variables
    QSharedPointer<ObjectState> mObjectState;
    RECEIVER_STATE mReceiverState = RECEIVER_STATE::UNKNOWN;
};

#endif // GNSSRECEIVER_H
