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
#include <QDateTime>
#include <QHostAddress>
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
#include "communication/mavlinkparameterserver.h"
#include <mavsdk/plugins/mission_raw/mission_raw.h>

class MavsdkVehicleServer : public QObject
{
    Q_OBJECT
public:
    explicit MavsdkVehicleServer(QSharedPointer<VehicleState> vehicleState, const QHostAddress controlTowerAddress = QHostAddress("127.0.0.1"),
                                 const unsigned controlTowerPort = 14540, const QAbstractSocket::SocketType controlTowerSocketType = QAbstractSocket::UdpSocket); // NOTE: currently, only UDP supported in mavsdk on vehicle side
    void setUbloxRover(QSharedPointer<UbloxRover> ubloxRover);
    void setWaypointFollower(QSharedPointer<WaypointFollower> waypointFollower);
    void setMovementController(QSharedPointer<MovementController> movementController);
    void setManualControlMaxSpeed(double manualControlMaxSpeed_ms);
    double getManualControlMaxSpeed() const;
    void mavResult(const uint16_t command, MAV_RESULT result);
    void sendGpsOriginLlh(const llh_t &gpsOriginLlh);
    void on_logSent(const QString& message, const quint8& severity);

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

    QDateTime mMavsdkVehicleServerCreationTime = QDateTime::currentDateTime();
    QSharedPointer<VehicleState> mVehicleState;
    QSharedPointer<UbloxRover> mUbloxRover;
    QSharedPointer<WaypointFollower> mWaypointFollower;
    QSharedPointer<MovementController> mMovementController;
    ParameterServer *mParameterServer;

    bool mHeartbeat;
    QTimer mHeartbeatTimer;
    const unsigned mCountdown_ms = 2000;

    void updateRawGpsAndGpsInfoFromUbx(const ubx_nav_pvt &pvt);
    void heartbeatTimeout();
    void heartbeatReset();
    PosPoint convertMissionItemToPosPoint(const mavsdk::MissionRawServer::MissionItem &item);
    void handleManualControlMessage(mavlink_manual_control_t manualControl);
    void sendMissionAck(quint8 type);
    double mManualControlMaxSpeed = 2.0; // [m/s]
};

#endif // MAVSDKVEHICLESERVER_H
