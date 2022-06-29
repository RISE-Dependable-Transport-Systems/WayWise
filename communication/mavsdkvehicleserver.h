#ifndef MAVSDKVEHICLESERVER_H
#define MAVSDKVEHICLESERVER_H

#include <QObject>
#include <QSharedPointer>
#include <QTimer>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/param_server/param_server.h>
#include <mavsdk/plugins/telemetry_server/telemetry_server.h>
#include <mavsdk/plugins/action_server/action_server.h>
#include <mavsdk/plugins/mission_raw_server/mission_raw_server.h>
#include <mavsdk/plugins/mission/mission.h>
#include <WayWise/vehicles/vehiclestate.h>
#include <WayWise/sensors/gnss/ubloxrover.h>

class MavsdkVehicleServer : public QObject
{
    Q_OBJECT
public:
    explicit MavsdkVehicleServer(QSharedPointer<VehicleState> vehicleState);
    void setUbloxRover(QSharedPointer<UbloxRover> ubloxRover);

signals:

private:
    mavsdk::Mavsdk mMavsdk;
    std::shared_ptr<mavsdk::TelemetryServer> mTelemetryServer;
    std::shared_ptr<mavsdk::ActionServer> mActionServer;
    std::shared_ptr<mavsdk::ParamServer> mParamServer;
    std::shared_ptr<mavsdk::MavlinkPassthrough> mMavlinkPassthrough;
    QTimer mPublishMavlinkTimer;

    QSharedPointer<VehicleState> mVehicleState;
    QSharedPointer<UbloxRover> mUbloxRover;

};

#endif // MAVSDKVEHICLESERVER_H
