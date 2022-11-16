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
#include "waywise.h"
#include "vehicleconnection.h"
#include "vehicles/vehiclestate.h"
#include "core/coordinatetransforms.h"
#include "autopilot/gotowaypointfollower.h"
//#include "autopilot/purepursuitwaypointfollower.h"
#include "vehicles/copterstate.h"
#include "vehicles/carstate.h"
#include "sensors/camera/mavsdkgimbal.h"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/mission_raw/mission_raw.h>
#include <mavsdk/plugins/info/info.h>

class MavsdkVehicleConnection : public VehicleConnection
{
    Q_OBJECT
public:
    explicit MavsdkVehicleConnection(std::shared_ptr<mavsdk::System> system, MAV_TYPE vehicleType);
    void setEnuReference(const llh_t &enuReference);
    void setHomeLlh(const llh_t &homeLlh);
    virtual void requestArm() override;
    virtual void requestDisarm() override;
    virtual void requestTakeoff() override;
    virtual void requestLanding() override;
    virtual void requestPrecisionLanding() override;
    virtual void requestReturnToHome() override;
    virtual void requestManualControl() override;
    virtual void requestFollowPoint() override;
    void requestGotoLlh(const llh_t &llh, bool changeFlightmodeToHold = false);
    virtual void requestGotoENU(const xyz_t &xyz, bool changeFlightmodeToHold = false) override;
    virtual void requestVelocityAndYaw(const xyz_t &velocityENU, const double &yawDeg) override;
    void inputRtcmData(const QByteArray &rtcmData);
    void sendLandingTargetLlh(const llh_t &landingTargetLlh);
    void sendLandingTargetENU(const xyz_t &landingTargetENU);
    void sendSetGpsOriginLlh(const llh_t &gpsOriginLlh);
    virtual void setActuatorOutput(int index, float value) override;
    virtual void setManualControl(double x, double y, double z, double r, uint16_t buttonStateMask) override;

    void setConvertLocalPositionsToGlobalBeforeSending(bool convertLocalPositionsToGlobalBeforeSending);

    MAV_TYPE getVehicleType() const;

signals:
    void gotVehicleHomeLlh(const llh_t &homePositionLlh);
    void stopWaypointFollowerSignal(); // Used internally from MAVSDK callbacks (that live in other threads)

private:
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
    std::shared_ptr<mavsdk::MissionRaw> mMissionRaw;
    QSharedPointer<QTimer> mPosTimer;

    mavsdk::MissionRaw::MissionItem convertPosPointToMissionItem(const PosPoint& posPoint, int sequenceId, bool current = false);

    // VehicleConnection interface
protected:
    virtual bool isAutopilotActiveOnVehicle() override;
    virtual void restartAutopilotOnVehicle() override;
    virtual void startAutopilotOnVehicle() override;
    virtual void pauseAutopilotOnVehicle() override;
    virtual void stopAutopilotOnVehicle() override;
    virtual void clearRouteOnVehicle(int id) override;
    virtual void appendToRouteOnVehicle(const QList<PosPoint> &route, int id) override;
};

#endif // MAVSDKVEHICLECONNECTION_H
