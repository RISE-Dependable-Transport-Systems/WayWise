/*
 *     Copyright 2022 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#include "gotowaypointfollower.h"

GotoWaypointFollower::GotoWaypointFollower(QSharedPointer<VehicleConnection> vehicleConnection, PosType posTypeUsed)
{
    mVehicleConnection = vehicleConnection;
    connect(&mUpdateStateTimer, &QTimer::timeout, this, &GotoWaypointFollower::updateState);
    setPosTypeUsed(posTypeUsed);
}

bool GotoWaypointFollower::getRepeatRoute() const
{
    return mCurrentState.repeatRoute;
}

void GotoWaypointFollower::setRepeatRoute(bool value)
{
    mCurrentState.repeatRoute = value;
}

const PosPoint GotoWaypointFollower::getCurrentGoal()
{
    return mCurrentState.currentGoal;
}

void GotoWaypointFollower::clearRoute()
{
    mWaypointList.clear();
}

void GotoWaypointFollower::addWaypoint(const PosPoint &point)
{
    mWaypointList.append(point);
}

void GotoWaypointFollower::addRoute(const QList<PosPoint> &route)
{
    mWaypointList.append(route);
}

void GotoWaypointFollower::startFollowingRoute(bool fromBeginning)
{
    mCurrentState.overrideAltitude = mVehicleConnection->getVehicleState()->getPosition(mPosTypeUsed).getHeight(); // Remove this line in order to use route height
    qDebug() << "Note: WaypointFollower starts following route. Height info from route is ignored (staying at" << QString::number(mCurrentState.overrideAltitude, 'g', 2) << "m).";

    if (fromBeginning || mCurrentState.stmState == GotoWayPointFollowerSTMstates::NONE) {
        mCurrentState.stmState = GotoWayPointFollowerSTMstates::FOLLOW_ROUTE_INIT;
        mUpdateStateTimer.start(mUpdateStatePeriod_ms);
    } else {
        mCurrentState.stmState = GotoWayPointFollowerSTMstates::FOLLOW_ROUTE_GOTO;
        mUpdateStateTimer.start(mUpdateStatePeriod_ms);
    }
}

bool GotoWaypointFollower::isActive()
{
    return mUpdateStateTimer.isActive();
}

void GotoWaypointFollower::stop()
{
    mUpdateStateTimer.stop();

    holdPosition();
}

void GotoWaypointFollower::resetState()
{
    mUpdateStateTimer.stop();

    mCurrentState.stmState = GotoWayPointFollowerSTMstates::NONE;
    mCurrentState.currentWaypointIndex = mWaypointList.size();
}

PosType GotoWaypointFollower::getPosTypeUsed() const
{
    return mPosTypeUsed;
}

void GotoWaypointFollower::setPosTypeUsed(const PosType &posTypeUsed)
{
    mPosTypeUsed = posTypeUsed;
}

double GotoWaypointFollower::getWaypointProximity() const
{
    return mCurrentState.waypointProximity;
}

void GotoWaypointFollower::setWaypointProximity(double value)
{
    mCurrentState.waypointProximity = value;
}

PosPoint GotoWaypointFollower::getCurrentVehiclePosition()
{
     return mVehicleConnection->getVehicleState()->getPosition(mPosTypeUsed);
}

void GotoWaypointFollower::holdPosition()
{
     mVehicleConnection->requestGotoENU({getCurrentVehiclePosition().getX(), getCurrentVehiclePosition().getY(), getCurrentVehiclePosition().getHeight()});
}

void GotoWaypointFollower::updateState()
{
    switch (mCurrentState.stmState) {
    case GotoWayPointFollowerSTMstates::NONE:
        qDebug() << "WARNING: WayPointFollower running uninitialized statemachine.";
        break;

    case GotoWayPointFollowerSTMstates::FOLLOW_ROUTE_INIT:
        if (mWaypointList.size()) {
            mCurrentState.currentWaypointIndex = 0;
            mCurrentState.currentGoal = mWaypointList.at(0);
            if (mCurrentState.overrideAltitude > 0)
                mCurrentState.currentGoal.setHeight(mCurrentState.overrideAltitude);
            mCurrentState.stmState = GotoWayPointFollowerSTMstates::FOLLOW_ROUTE_GOTO;
        } else {
            mCurrentState.stmState = GotoWayPointFollowerSTMstates::FOLLOW_ROUTE_FINISHED;
            qDebug() << "Route is missing. Waypointfollower stopped";
        }
        break;

    case GotoWayPointFollowerSTMstates::FOLLOW_ROUTE_GOTO:
            mVehicleConnection->requestGotoENU({mCurrentState.currentGoal.getX(), mCurrentState.currentGoal.getY(), mCurrentState.currentGoal.getHeight()});
            mCurrentState.stmState = GotoWayPointFollowerSTMstates::FOLLOWING_ROUTE;
        break;

    case GotoWayPointFollowerSTMstates::FOLLOWING_ROUTE:
        if (getCurrentVehiclePosition().getDistanceTo3d(mCurrentState.currentGoal) < mCurrentState.waypointProximity){ // Consider previous waypoint as reached
            mCurrentState.stmState = GotoWayPointFollowerSTMstates::FOLLOW_ROUTE_HOLD_POSITION;
            qDebug() << "Holding position for: " << mUpdateWaypointPeriod_ms*0.001 << " seconds";
        } else
            qDebug() << "Distance to waypoint: " << getCurrentVehiclePosition().getDistanceTo3d(mCurrentState.currentGoal) << " [m]";
        break;

    case GotoWayPointFollowerSTMstates::FOLLOW_ROUTE_HOLD_POSITION:
        // Holding position for mUpdateWaypointPeriod_ms
        if (mUpdateStateSumator >= mUpdateWaypointPeriod_ms) {
            mUpdateStateSumator = 0;
            mCurrentState.currentWaypointIndex++;
            if (mCurrentState.currentWaypointIndex != mWaypointList.size()) {
                mCurrentState.currentGoal = mWaypointList.at(mCurrentState.currentWaypointIndex);
                if (mCurrentState.overrideAltitude > 0)
                    mCurrentState.currentGoal.setHeight(mCurrentState.overrideAltitude);
                mCurrentState.stmState = GotoWayPointFollowerSTMstates::FOLLOW_ROUTE_GOTO;
            } else {
                if (mCurrentState.repeatRoute)
                    mCurrentState.stmState = GotoWayPointFollowerSTMstates::FOLLOW_ROUTE_INIT;
                else
                    mCurrentState.stmState = GotoWayPointFollowerSTMstates::FOLLOW_ROUTE_FINISHED;
            }
        } else
            mUpdateStateSumator+=mUpdateStatePeriod_ms;
        break;

    case GotoWayPointFollowerSTMstates::FOLLOW_ROUTE_FINISHED:
        mCurrentState.stmState = GotoWayPointFollowerSTMstates::NONE;
        mCurrentState.currentWaypointIndex = mWaypointList.size();
        stop();
        qDebug() << "Route finished";
        break;

    default:
        break;
    }
}

QList<PosPoint> GotoWaypointFollower::getCurrentRoute()
{
    return mWaypointList;
}
