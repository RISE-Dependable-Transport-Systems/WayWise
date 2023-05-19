/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *               2022 Rickard Häll      rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#ifndef MAVSDKVEHICLESERVER_H
#define MAVSDKVEHICLESERVER_H

#include <QObject>
#include <QSharedPointer>
#include <QTimer>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action_server/action_server.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/mission_raw_server/mission_raw_server.h>
#include <mavsdk/plugins/telemetry_server/telemetry_server.h>
#include <mavsdk/server_component.h>
#include "autopilot/waypointfollower.h"
#include "sensors/gnss/ubloxrover.h"
#include "vehicles/controller/movementcontroller.h"
#include "vehicles/vehiclestate.h"
#include "waywise.h"
#include "communication/parameterserver.h"

class MavsdkVehicleServer : public QObject
{
    Q_OBJECT
public:
    explicit MavsdkVehicleServer(QSharedPointer<VehicleState> vehicleState);
    void setUbloxRover(QSharedPointer<UbloxRover> ubloxRover);
    void setWaypointFollower(QSharedPointer<WaypointFollower> waypointFollower);
    void setMovementController(QSharedPointer<MovementController> movementController);
    void setManualControlMaxSpeed(double manualControlMaxSpeed_ms);
    void mavResult(const uint16_t command, MAV_RESULT result);
    void sendGpsOriginLlh(const llh_t &gpsOriginLlh);
    QSharedPointer<ParameterServer> getParameterServer();

signals:
    void startWaypointFollower(bool fromBeginning); // to enable starting from MAVSDK thread
    void pauseWaypointFollower();
    void resetWaypointFollower();
    void clearRouteOnWaypointFollower();
    void startFollowPoint();
    void resetHeartbeat();
    void switchAutopilotID(const float autopilotID);
    void rxRtcmData(const QByteArray rtcmData);
    void shutdownOrRebootOnboardComputer(bool isShutdown);

private:
    mavsdk::Mavsdk mMavsdk;
    std::shared_ptr<mavsdk::TelemetryServer> mTelemetryServer;
    mavsdk::TelemetryServer::RawGps mRawGps;
    mavsdk::TelemetryServer::GpsInfo mGpsInfo {0, mavsdk::TelemetryServer::FixType::NoGps};
    std::shared_ptr<mavsdk::ActionServer> mActionServer;
    std::shared_ptr<mavsdk::MissionRawServer> mMissionRawServer;
    std::shared_ptr<mavsdk::MavlinkPassthrough> mMavlinkPassthrough;
    QTimer mPublishMavlinkTimer;

    QSharedPointer<VehicleState> mVehicleState;
    QSharedPointer<UbloxRover> mUbloxRover;
    QSharedPointer<WaypointFollower> mWaypointFollower;
    QSharedPointer<MovementController> mMovementController;
    QSharedPointer<ParameterServer> mParameterServer;

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
