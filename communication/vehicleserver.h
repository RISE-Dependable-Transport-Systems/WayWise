/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *               2022 Rickard Häll      rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#ifndef VEHICLESERVER_H
#define VEHICLESERVER_H

#include <QObject>
#include <QSharedPointer>
#include <QTimer>
#include "autopilot/waypointfollower.h"
#include "autopilot/followpoint.h"
#include "sensors/gnss/ubloxrover.h"
#include "vehicles/controller/movementcontroller.h"
#include "vehicles/vehiclestate.h"
#include "waywise.h"

class VehicleServer : public QObject
{
    Q_OBJECT
public:
    VehicleServer(QSharedPointer<VehicleState> vehicleState) : mVehicleState(vehicleState) {};
    virtual void setUbloxRover(QSharedPointer<UbloxRover> ubloxRover) = 0;
    virtual void setGNSSReceiver(QSharedPointer<GNSSReceiver> gnssReceiver) { mGNSSReceiver = gnssReceiver; }
    virtual void setWaypointFollower(QSharedPointer<WaypointFollower> waypointFollower) = 0;
    virtual void setMovementController(QSharedPointer<MovementController> movementController) = 0;
    virtual void setManualControlMaxSpeed(double manualControlMaxSpeed_ms) = 0;
    virtual void setFollowPoint(QSharedPointer<FollowPoint> followPoint) = 0;
    virtual double getManualControlMaxSpeed() const = 0;
    virtual void sendGpsOriginLlh(const llh_t &gpsOriginLlh) = 0;
    virtual void updateRawGpsAndGpsInfoFromUbx(const ubx_nav_pvt &pvt) = 0;

signals:
    void startWaypointFollower(bool fromBeginning);
    void pauseWaypointFollower();
    void resetWaypointFollower();
    void clearRouteOnWaypointFollower();
    void startFollowPoint();
    void stopFollowPoint();
    void resetHeartbeat();
    void switchAutopilotID(const float autopilotID);
    void rxRtcmData(const QByteArray rtcmData);
    void shutdownOrRebootOnboardComputer(bool isShutdown);

protected:
    QSharedPointer<VehicleState> mVehicleState;
    QSharedPointer<GNSSReceiver> mGNSSReceiver;
    QSharedPointer<WaypointFollower> mWaypointFollower;
    QSharedPointer<MovementController> mMovementController;
    QSharedPointer<FollowPoint> mFollowPoint;

    bool mHeartbeat;
    QTimer mHeartbeatTimer;
    const unsigned mCountdown_ms = 2000;

    virtual void heartbeatTimeout() = 0;
    virtual void heartbeatReset() = 0;

    double mManualControlMaxSpeed = 2.0; // [m/s]
};

#endif // VEHICLESERVER_H
