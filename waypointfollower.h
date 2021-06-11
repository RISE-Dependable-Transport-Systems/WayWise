#ifndef WAYPOINTFOLLOWER_H
#define WAYPOINTFOLLOWER_H

#include <QObject>
#include <QSharedPointer>
#include <QPointF>
#include <QTimer>
#include "vehiclestate.h"
#include "movementcontroller.h"

enum WayPointFollowerSTMstates {NONE, FOLLOW_POINT_FOLLOWING, FOLLOW_POINT_WAITING, FOLLOW_ROUTE_INIT, FOLLOW_ROUTE_GOTO_BEGIN, FOLLOW_ROUTE_FOLLOWING, FOLLOW_ROUTE_FINISHED};
struct WayPointFollowerState {
    WayPointFollowerSTMstates stmState = NONE;
    PosPoint currentGoal;
    int currentWaypointIndex;
    double purePursuitRadius = 1.0;
    int numWaypointsLookahead = 8;
};

class WaypointFollower : public QObject
{
    Q_OBJECT
public:
    WaypointFollower(QSharedPointer<MovementController> movementController);

    double getPurePursuitRadius() const;
    void setPurePursuitRadius(double value);

    int getCurrentWaypointindex();
    void setCurrentWaypointindex(int value);

    PosPoint getCurrentGoal();
    void setCurrentGoal(PosPoint &point);

    WayPointFollowerSTMstates getSTMState();
    void setSTMState(WayPointFollowerSTMstates state);

    void clearRoute();
    void addWaypoint(const PosPoint &point);

    void startFollowingRoute(bool fromBeginning);
    void stopFollowingRoute();
    void resetState();

    static double getCurvatureToPoint(QSharedPointer<VehicleState> vehicleState, const QPointF& point, PosType vehiclePosType = PosType::simulated);
    double getCurvatureToPoint(const QPointF& point);

    double getInterpolatedSpeed(const PosPoint &currentGoal, const PosPoint &lastWaypoint, const PosPoint &nextWaypoint);

    PosType getPosTypeUsed() const;
    void setPosTypeUsed(const PosType &posTypeUsed);

signals:

private:
    void updateState();
    WayPointFollowerState mCurrentState;

    PosType mPosTypeUsed = PosType::fused; // The type of position (Odom, GNSS, UWB, ...) that should be used for planning
    QSharedPointer<MovementController> mMovementController;
    QList <PosPoint> mWaypointList;
    const unsigned mUpdateStatePeriod_ms = 50;
    QTimer mUpdateStateTimer;

};

#endif // WAYPOINTFOLLOWER_H
