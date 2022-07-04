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
#include "vehicles/vehiclestate.h"
#include "sensors/gnss/ubloxrover.h"
#include "autopilot/waypointfollower.h"
#include "waywise.h"

class MavsdkVehicleServer : public QObject
{
    Q_OBJECT
public:
    explicit MavsdkVehicleServer(QSharedPointer<VehicleState> vehicleState);
    void setUbloxRover(QSharedPointer<UbloxRover> ubloxRover);
    void setWaypointFollower(QSharedPointer<WaypointFollower> waypointFollower);

signals:
    void startWaypointFollower(bool fromBeginning); // to enable starting from MAVSDK thread
    void pauseWaypointFollower();
    void resetWaypointFollower();
    void clearRouteOnWaypointFollower();

private:
    mavsdk::Mavsdk mMavsdk;
    std::shared_ptr<mavsdk::TelemetryServer> mTelemetryServer;
    std::shared_ptr<mavsdk::ActionServer> mActionServer;
    std::shared_ptr<mavsdk::ParamServer> mParamServer;
    std::shared_ptr<mavsdk::MissionRawServer> mMissionRawServer;
    QTimer mPublishMavlinkTimer;

    QSharedPointer<VehicleState> mVehicleState;
    QSharedPointer<UbloxRover> mUbloxRover;
    QSharedPointer<WaypointFollower> mWaypointFollower;

    PosPoint convertMissionItemToPosPoint(const mavsdk::MissionRawServer::MissionItem &item);

};

#endif // MAVSDKVEHICLESERVER_H
