/*
 *     Copyright 2022 Rickard Häll   rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#include "multiwaypointfollower.h"

MultiWaypointFollower::MultiWaypointFollower(QSharedPointer<WaypointFollower> waypointfollower)
{
    mWayPointFollowerList.append(waypointfollower);
}

bool MultiWaypointFollower::getRepeatRoute() const
{
    return mWayPointFollowerList[mRouteID]->getRepeatRoute();
}

void MultiWaypointFollower::setRepeatRoute(bool value)
{
    mWayPointFollowerList[mRouteID]->setRepeatRoute(value);
}

const PosPoint MultiWaypointFollower::getCurrentGoal()
{
    return mWayPointFollowerList[mRouteID]->getCurrentGoal();
}

void MultiWaypointFollower::clearRoute()
{
    mWayPointFollowerList[mRouteID]->clearRoute();
}

void MultiWaypointFollower::addWaypoint(const PosPoint &point)
{
    mWayPointFollowerList[mRouteID]->addWaypoint(point);
}

void MultiWaypointFollower::addRoute(const QList<PosPoint> &route)
{
    mWayPointFollowerList[mRouteID]->addRoute(route);
}

void MultiWaypointFollower::startFollowingRoute(bool fromBeginning)
{
    mWayPointFollowerList[mRouteID]->startFollowingRoute(fromBeginning);
}

bool MultiWaypointFollower::isActive()
{
    return mWayPointFollowerList[mRouteID]->isActive();
}

void MultiWaypointFollower::stop()
{
    mWayPointFollowerList[mRouteID]->stop();
}

void MultiWaypointFollower::resetState()
{
    mWayPointFollowerList[mRouteID]->resetState();
}

void MultiWaypointFollower::addWaypointFollower(QSharedPointer<WaypointFollower> waypointfollower)
{
    mWayPointFollowerList.append(waypointfollower);
}

void MultiWaypointFollower::setActiveRoute(int routeID)
{
    mRouteID = routeID;
}

QSharedPointer<WaypointFollower> MultiWaypointFollower::getActiveWaypointFollower()
{
    return mWayPointFollowerList[mRouteID];
}

int MultiWaypointFollower::getNumberOfRoutes()
{
    return mWayPointFollowerList.size();
}
