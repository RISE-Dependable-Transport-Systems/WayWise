#ifndef WAYPOINTFOLLOWER_H
#define WAYPOINTFOLLOWER_H

#include <QObject>
#include <QSharedPointer>
#include <QPointF>
#include <QTimer>
#include "vehiclestate.h"
#include "movementcontroller.h"

class WaypointFollower : public QObject
{
    Q_OBJECT
public:
    WaypointFollower(QSharedPointer<MovementController> movementController);

    double getLookahead() const;
    void setLookahead(double getLookahead);

    void clearRoute();
    void addWaypoint(const PosPoint &point);

    void startFollowingRoute(bool fromBeginning);
    void stopFollowingRoute();
    void resetState();

    static double getCurvatureToPoint(QSharedPointer<VehicleState> vehicleState, const QPointF& point);
    double getCurvatureToPoint(const QPointF& point);

signals:

private:
    enum FollowSTMstates {NONE, FOLLOW_POINT_FOLLOWING, FOLLOW_POINT_WAITING, FOLLOW_ROUTE_INIT, FOLLOW_ROUTE_GOTO_BEGIN, FOLLOW_ROUTE_FOLLOWING, FOLLOW_ROUTE_END};
    struct WayPointFollowerState {
        FollowSTMstates stmState = NONE;
        PosPoint currentGoal;
        unsigned currentWaypoint;
    } mCurrentState;
    void updateState();

    QSharedPointer<MovementController> mMovementController;
    double mLookahead = 1.0;
    QList <PosPoint> mWaypointList;
    const unsigned mUpdateStatePeriod_ms = 100;
    QTimer mUpdateStateTimer;

};

#endif // WAYPOINTFOLLOWER_H
