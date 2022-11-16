/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#ifndef MAVSDKVEHICLESERVER_H
#define MAVSDKVEHICLESERVER_H

#include <QObject>
#include <QSharedPointer>
#include <QTimer>
#include <mavsdk/mavsdk.h>
#include <mavsdk/server_component.h>
#include <mavsdk/plugins/param_server/param_server.h>
#include <mavsdk/plugins/telemetry_server/telemetry_server.h>
#include <mavsdk/plugins/action_server/action_server.h>
#include <mavsdk/plugins/mission_raw_server/mission_raw_server.h>
#include "waywise.h"
#include "vehicles/vehiclestate.h"
#include "sensors/gnss/ubloxrover.h"
#include "autopilot/waypointfollower.h"
#include "vehicles/controller/movementcontroller.h"

class MavsdkVehicleServer : public QObject
{
    Q_OBJECT
public:
    explicit MavsdkVehicleServer(QSharedPointer<VehicleState> vehicleState);
    void setUbloxRover(QSharedPointer<UbloxRover> ubloxRover);
    void setWaypointFollower(QSharedPointer<WaypointFollower> waypointFollower);
    void setMovementController(QSharedPointer<MovementController> movementController);
    void setManualControlMaxSpeed(double manualControlMaxSpeed_ms);

signals:
    void startWaypointFollower(bool fromBeginning); // to enable starting from MAVSDK thread
    void pauseWaypointFollower();
    void resetWaypointFollower();
    void clearRouteOnWaypointFollower();
    void startFollowPoint();
    void resetHeartbeat();

private:
    mavsdk::Mavsdk mMavsdk;
    std::shared_ptr<mavsdk::TelemetryServer> mTelemetryServer;
    mavsdk::TelemetryServer::RawGps mRawGps;
    mavsdk::TelemetryServer::GpsInfo mGpsInfo {0, mavsdk::TelemetryServer::FixType::NoGps};
    std::shared_ptr<mavsdk::ActionServer> mActionServer;
    std::shared_ptr<mavsdk::ParamServer> mParamServer;
    std::shared_ptr<mavsdk::MissionRawServer> mMissionRawServer;
    QTimer mPublishMavlinkTimer;

    QSharedPointer<VehicleState> mVehicleState;
    QSharedPointer<UbloxRover> mUbloxRover;
    QSharedPointer<WaypointFollower> mWaypointFollower;
    QSharedPointer<MovementController> mMovementController;

    bool mHeartbeat;
    QTimer mHeartbeatTimer;
    const unsigned mCountdown_ms = 2000;

    void updateRawGpsAndGpsInfoFromUbx(const ubx_nav_pvt &pvt);
    void heartbeatTimeout();
    void heartbeatReset();
    PosPoint convertMissionItemToPosPoint(const mavsdk::MissionRawServer::MissionItem &item);
    void handleManualControlMessage(mavlink_manual_control_t manualControl);
    double mManualControlMaxSpeed = 2.0; // [m/s]

};

#endif // MAVSDKVEHICLESERVER_H
