/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Implementation of VehicleConnection to communicate with MAVLINK-based vehicles using MAVSDK
 * NOTE: Only multicopters supported for now.
 */

#ifndef MAVSDKVEHICLECONNECTION_H
#define MAVSDKVEHICLECONNECTION_H

#include <QSharedPointer>
#include "vehicleconnection.h"
#include "vehicles/vehiclestate.h"
#include "core/coordinatetransforms.h"
#include "autopilot/waypointfollower.h"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/offboard/offboard.h>

class MavsdkVehicleConnection : public VehicleConnection
{
    Q_OBJECT
public:
    explicit MavsdkVehicleConnection(std::shared_ptr<mavsdk::System> system);
    void setEnuReference(const llh_t &enuReference);
    void setHomeLlh(const llh_t &homeLlh);
    void requestArm();
    void requestDisarm();
    void requestTakeoff();
    void requestLanding();
    void requestPrecisionLanding();
    void requestReturnToHome();
    void requestGotoLlh(const llh_t &llh, bool changeFlightmodeToHold = false);
    virtual void requestGotoENU(const xyz_t &xyz, bool changeFlightmodeToHold = false) override;
    virtual void requestVelocityAndYaw(const xyz_t &velocityENU, const double &yawDeg) override;
    void inputRtcmData(const QByteArray &rtcmData);
    void sendLandingTargetLlh(const llh_t &landingTargetLlh);
    void sendLandingTargetENU(const xyz_t &landingTargetENU);
    void sendSetGpsOriginLlh(const llh_t &gpsOriginLlh);
    virtual void setActuatorOutput(int index, float value) override;

    void setConvertLocalPositionsToGlobalBeforeSending(bool convertLocalPositionsToGlobalBeforeSending);

    void setWaypointFollower(const QSharedPointer<WaypointFollower> &waypointFollower);
    QSharedPointer<WaypointFollower> getWaypointFollower() const;
    bool hasWaypointFollower();

signals:
    void gotVehicleHomeLlh(const llh_t &homePositionLlh);
    void stopWaypointFollowerSignal(); // Used internally from MAVSDK callbacks (that live in other threads)

private:
    void stopWaypointFollower();

    MAV_TYPE mVehicleType;
    llh_t mEnuReference;
    llh_t mGpsGlobalOrigin; // reference for on-vehicle EKF (origin in NED, ENU frames on vehicle), polled once at startup
                            // do not use unless you really know what you are doing!
                            // Use mEnuReference instead.
    bool mConvertLocalPositionsToGlobalBeforeSending = false;
    std::shared_ptr<mavsdk::System> mSystem;
    std::shared_ptr<mavsdk::Telemetry> mTelemetry;
    std::shared_ptr<mavsdk::Action> mAction;
    std::shared_ptr<mavsdk::Param> mParam;
    std::shared_ptr<mavsdk::MavlinkPassthrough> mMavlinkPassthrough;
    std::shared_ptr<mavsdk::Offboard> mOffboard;
    QSharedPointer<QTimer> mPosTimer;
    QSharedPointer<WaypointFollower> mWaypointFollower;
};

#endif // MAVSDKVEHICLECONNECTION_H
