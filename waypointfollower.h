#ifndef WAYPOINTFOLLOWER_H
#define WAYPOINTFOLLOWER_H

#include <QObject>
#include <QSharedPointer>
#include <QPointF>
#include <QTimer>
#include "vehiclestate.h"
#include "movementcontroller.h"
#include "sdvp_qtcommon/legacy/datatypes.h"

enum WayPointFollowerSTMstates {NONE, FOLLOW_POINT_FOLLOWING, FOLLOW_POINT_WAITING, FOLLOW_ROUTE_INIT, FOLLOW_ROUTE_GOTO_BEGIN, FOLLOW_ROUTE_FOLLOWING, FOLLOW_ROUTE_FINISHED};
struct WayPointFollowerState {
    WayPointFollowerSTMstates stmState = NONE;
    PosPoint currentGoal;
    int currentWaypointIndex;
    double purePursuitRadius = 1.0;
    // Follow Route
    int numWaypointsLookahead = 8;
    bool repeatRoute = false;
    // Follow Point
    PosPoint currentFollowPoint; // always in ENU
    double followPointSpeed = 1.5;
    bool followPointTimedOut = true;
};

class WaypointFollower : public QObject
{
    Q_OBJECT
public:
    WaypointFollower(QSharedPointer<MovementController> movementController);

    double getPurePursuitRadius() const;
    void setPurePursuitRadius(double value);

    double getFollowPointSpeed() const;
    void setFollowPointSpeed(double value);

    bool getRepeatRoute() const;
    void setRepeatRoute(bool value);

    const PosPoint getCurrentGoal();

    void clearRoute();
    void addWaypoint(const PosPoint &point);

    void startFollowingRoute(bool fromBeginning);
    bool isActive();
    void stop();
    void resetState();

    void startFollowPoint();

    static double getCurvatureToPointInENU(QSharedPointer<VehicleState> vehicleState, const QPointF& point, PosType vehiclePosType = PosType::simulated);
    double getCurvatureToPointInENU(const QPointF& point);

    double getInterpolatedSpeed(const PosPoint &currentGoal, const PosPoint &lastWaypoint, const PosPoint &nextWaypoint);

    PosType getPosTypeUsed() const;
    void setPosTypeUsed(const PosType &posTypeUsed);

signals:

public slots:
    void updateFollowPoint(const PosPoint &point);

private:
    const unsigned mFollowPointTimeout_ms = 1000;
    QTimer mFollowPointHeartbeatTimer;

    void updateState();
    WayPointFollowerState mCurrentState;

    PosType mPosTypeUsed = PosType::fused; // The type of position (Odom, GNSS, UWB, ...) that should be used for planning
    QSharedPointer<MovementController> mMovementController;
    QList <PosPoint> mWaypointList;
    const unsigned mUpdateStatePeriod_ms = 50;
    QTimer mUpdateStateTimer;

};

#endif // WAYPOINTFOLLOWER_H
