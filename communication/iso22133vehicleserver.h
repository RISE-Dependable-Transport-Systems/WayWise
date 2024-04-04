
#include <WayWise/communication/vehicleserver.h>
#include <QObject>
#include <QSharedPointer>
#include <QTimer>
#include "iso22133object.hpp"

#pragma once

class iso22133VehicleServer : public VehicleServer, public ISO22133::TestObject
{
    Q_OBJECT
public:
    iso22133VehicleServer(QSharedPointer<VehicleState> vehicleState, const std::string &ip);
    void setUbloxRover(QSharedPointer<UbloxRover> ubloxRover) override;
    void setWaypointFollower(QSharedPointer<WaypointFollower> waypointFollower) override;
    void setMovementController(QSharedPointer<MovementController> movementController) override;
    void setManualControlMaxSpeed(double manualControlMaxSpeed_ms) override;
    double getManualControlMaxSpeed() const override;
    void sendGpsOriginLlh(const llh_t &gpsOriginLlh) override;

    // ISO22133 overrides
    void handleAbort() override;
    void onOSEM(ObjectSettingsType& osem) override;
    void onTRAJ() override;
    void onSTRT(StartMessageType&) override;

private: 
    void setMonr(CartesianPosition pos, SpeedType spd, AccelerationType acc, DriveDirectionType drd);
    QTimer mSetMonrTimer;
    void updateRawGpsAndGpsInfoFromUbx(const ubx_nav_pvt &pvt) override;
    void heartbeatTimeout() override;
    void heartbeatReset() override;
    void setStopCommands();
    static PosPoint convertTrajPointToPosPoint(const TrajectoryWaypointType &trajPoint);
};