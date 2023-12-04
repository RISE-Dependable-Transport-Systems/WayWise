/*
 *     Copyright 2022 Rickard Häll   rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Implementation of WaypointFollower to control autopilot through VehicleConnection
 */

#ifndef GOTOWAYPOINTFOLLOWER_H
#define GOTOWAYPOINTFOLLOWER_H

#include <QSharedPointer>
#include <QTimer>
#include "autopilot/waypointfollower.h"
#include "communication/vehicleconnections/vehicleconnection.h"

enum class GotoWayPointFollowerSTMstates {NONE, FOLLOW_ROUTE_INIT, FOLLOW_ROUTE_GOTO, FOLLOWING_ROUTE, FOLLOW_ROUTE_HOLD_POSITION, FOLLOW_ROUTE_FINISHED};
struct GotoWayPointFollowerState {
    GotoWayPointFollowerSTMstates stmState = GotoWayPointFollowerSTMstates::NONE;
    PosPoint currentGoal;
    int currentWaypointIndex;
    double waypointProximity = 3.0; // [m]
    // Follow Route
    bool repeatRoute = false;
    // -- for flying vehicles
    double overrideAltitude = 0.0; // [m]
};

class GotoWaypointFollower : public WaypointFollower
{
    Q_OBJECT
public:
    GotoWaypointFollower(QSharedPointer<VehicleConnection> vehicleConnection, PosType posTypeUsed);

    virtual bool getRepeatRoute() const override;
    virtual void setRepeatRoute(bool value) override;

    virtual const PosPoint getCurrentGoal() override;

    virtual void clearRoute() override;
    virtual void addWaypoint(const PosPoint &point) override;
    virtual void addRoute(const QList<PosPoint>& route) override;
    virtual QList<PosPoint> getCurrentRoute() override;

    virtual void startFollowingRoute(bool fromBeginning) override;
    virtual bool isActive() override;
    virtual void stop() override;
    virtual void resetState() override;

    PosType getPosTypeUsed() const;
    void setPosTypeUsed(const PosType &posTypeUsed);

    double getWaypointProximity() const;
    void setWaypointProximity(double value);

private:
    GotoWayPointFollowerState mCurrentState;
    PosType mPosTypeUsed = PosType::fused; // The type of position (Odom, GNSS, UWB, ...) that should be used for planning
    QSharedPointer<VehicleConnection> mVehicleConnection;
    QList <PosPoint> mWaypointList;
    unsigned mUpdateStatePeriod_ms = 200;
    QTimer mUpdateStateTimer;
    unsigned mUpdateWaypointPeriod_ms = 5000;
    unsigned mUpdateStateSumator = 0;

    PosPoint getCurrentVehiclePosition();
    void holdPosition();
    void updateState();
};

#endif // GOTOWAYPOINTFOLLOWER_H
