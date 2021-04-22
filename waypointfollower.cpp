#include "waypointfollower.h"
#include <cmath>
#include <QDebug>

WaypointFollower::WaypointFollower(QSharedPointer<MovementController> movementController)
{
    mMovementController = movementController;
    connect(&mUpdateStateTimer, &QTimer::timeout, this, &WaypointFollower::updateState);
}

double WaypointFollower::getLookahead() const
{
    return mLookahead;
}

void WaypointFollower::setLookahead(double lookahead)
{
    mLookahead = lookahead;
}

void WaypointFollower::clearRoute()
{
    mWaypointList.clear();
}

void WaypointFollower::addWaypoint(const PosPoint &point)
{
    mWaypointList.append(point);
}

void WaypointFollower::startFollowingRoute(bool fromBeginning)
{
    if (fromBeginning)
        resetState();

    mCurrentState.stmState = FOLLOW_ROUTE_INIT;
    mUpdateStateTimer.start(mUpdateStatePeriod_ms);
}

void WaypointFollower::stopFollowingRoute()
{
    mUpdateStateTimer.stop();
    // TODO: brake
}

void WaypointFollower::resetState()
{
    mUpdateStateTimer.stop();

    mCurrentState.stmState = FollowSTMstates::NONE;
    mCurrentState.currentWaypoint = mWaypointList.size();
}

double WaypointFollower::getCurvatureToPoint(QSharedPointer<VehicleState> vehicleState, const QPointF &point)
{
    // vehicleState and point in ENU frame
    // 1. transform point to vehicle frame, TODO: general transform in vehicleState?
    QPointF pointInVehicleFrame;
    // translate
    pointInVehicleFrame.setX(point.x()-vehicleState->getPosition().getX());
    pointInVehicleFrame.setY(point.y()-vehicleState->getPosition().getY());
    // rotate
    double currYaw = vehicleState->getPosition().getYaw();
    const double newX = cos(currYaw)*pointInVehicleFrame.x() - sin(currYaw)*pointInVehicleFrame.y();
    const double newY = sin(currYaw)*pointInVehicleFrame.x() + cos(currYaw)*pointInVehicleFrame.y();
    pointInVehicleFrame.setX(newX);
    pointInVehicleFrame.setY(newY);

    // 2. calc steering angle (pure pursuit)
    double distanceSquared = pow(pointInVehicleFrame.x(), 2) + pow(pointInVehicleFrame.y(), 2);
    double steeringAngleProportional = (2*pointInVehicleFrame.y()) / distanceSquared;
    return -steeringAngleProportional;
}

double WaypointFollower::getCurvatureToPoint(const QPointF &point)
{
    return getCurvatureToPoint(mMovementController->getVehicleState(), point);
}

void WaypointFollower::updateState()
{
    switch (mCurrentState.stmState) {
    case WaypointFollower::NONE:
        qDebug() << "WARNING: WayPointFollower running uninitialized statemachine.";
        break;

    // FOLLOW_POINT: we follow a point that is moving "follow me"
    case WaypointFollower::FOLLOW_POINT_FOLLOWING:
        // TODO
        break;

    case WaypointFollower::FOLLOW_POINT_WAITING:
        // TODO
        break;

    // FOLLOW_ROUTE: waypoints describe a route to be followed waypoint by waypoint
    case WaypointFollower::FOLLOW_ROUTE_INIT:
        if (mWaypointList.size()) {
            mCurrentState.currentWaypoint = 0;
            mCurrentState.currentGoal = mWaypointList.at(0);
            mCurrentState.stmState = FOLLOW_ROUTE_GOTO_BEGIN;
        } else
            mCurrentState.stmState = FOLLOW_ROUTE_END;
        break;

    case WaypointFollower::FOLLOW_ROUTE_GOTO_BEGIN:
        mMovementController->setDesiredSteering(getCurvatureToPoint(mCurrentState.currentGoal.getPoint())); // TODO: steering should be proportional to curvature (but not necessarily equal)
        mMovementController->setDesiredSpeed(mCurrentState.currentGoal.getSpeed());
        // TODO: -> FOLLOW_ROUTE_FOLLOWING when close to WP
        break;

    case WaypointFollower::FOLLOW_ROUTE_FOLLOWING:
        // TODO
        break;

    case WaypointFollower::FOLLOW_ROUTE_END:
        // TODO
        break;

    }
}
