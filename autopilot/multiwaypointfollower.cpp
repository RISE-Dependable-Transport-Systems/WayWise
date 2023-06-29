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
    return mWayPointFollowerList[mWaypointFollowerID]->getRepeatRoute();
}

void MultiWaypointFollower::setRepeatRoute(bool value)
{
    mWayPointFollowerList[mWaypointFollowerID]->setRepeatRoute(value);
}

const PosPoint MultiWaypointFollower::getCurrentGoal()
{
    return mWayPointFollowerList[mWaypointFollowerID]->getCurrentGoal();
}

void MultiWaypointFollower::clearRoute()
{
    mWayPointFollowerList[mWaypointFollowerID]->clearRoute();
}

void MultiWaypointFollower::addWaypoint(const PosPoint &point)
{
    mWayPointFollowerList[mWaypointFollowerID]->addWaypoint(point);
}

void MultiWaypointFollower::addRoute(const QList<PosPoint> &route)
{
    mWayPointFollowerList[mWaypointFollowerID]->addRoute(route);
}

void MultiWaypointFollower::startFollowingRoute(bool fromBeginning)
{
    mWayPointFollowerList[mWaypointFollowerID]->startFollowingRoute(fromBeginning);
}

bool MultiWaypointFollower::isActive()
{
    return mWayPointFollowerList[mWaypointFollowerID]->isActive();
}

void MultiWaypointFollower::stop()
{
    mWayPointFollowerList[mWaypointFollowerID]->stop();
}

void MultiWaypointFollower::resetState()
{
    mWayPointFollowerList[mWaypointFollowerID]->resetState();
}

double MultiWaypointFollower::getPurePursuitRadius() const
{
    if (qSharedPointerDynamicCast<PurepursuitWaypointFollower>(mWayPointFollowerList[mWaypointFollowerID]).get())
        return qSharedPointerCast<PurepursuitWaypointFollower>(mWayPointFollowerList[mWaypointFollowerID])->getPurePursuitRadius();
    else
        return 0;
}

void MultiWaypointFollower::setPurePursuitRadius(double value)
{
    for (const auto &wayPointFollower : mWayPointFollowerList)
        if(qSharedPointerDynamicCast<PurepursuitWaypointFollower>(wayPointFollower))
            qSharedPointerCast<PurepursuitWaypointFollower>(wayPointFollower)->setPurePursuitRadius(value);
        else
            qDebug() << "WaypointFollower has no function named setPurePursuitRadius";
}

void MultiWaypointFollower::setAdaptivePurePursuitRadiusActive(bool adaptive)
{
    for (const auto &wayPointFollower : mWayPointFollowerList)
        if(qSharedPointerDynamicCast<PurepursuitWaypointFollower>(wayPointFollower))
            qSharedPointerCast<PurepursuitWaypointFollower>(wayPointFollower)->setAdaptivePurePursuitRadiusActive(adaptive);
        else
            qDebug() << "WaypointFollower has no function named setAdaptivePurePursuitRadiusActive";
}

double MultiWaypointFollower::getAdaptivePurePursuitRadiusCoefficient()
{
    if (qSharedPointerDynamicCast<PurepursuitWaypointFollower>(mWayPointFollowerList[mWaypointFollowerID]).get())
        return qSharedPointerCast<PurepursuitWaypointFollower>(mWayPointFollowerList[mWaypointFollowerID])->getAdaptivePurePursuitRadiusCoefficient();
    else
        return 0;
}

void MultiWaypointFollower::setAdaptivePurePursuitRadiusCoefficient(double coefficient)
{
    for (const auto &wayPointFollower : mWayPointFollowerList)
        if(qSharedPointerDynamicCast<PurepursuitWaypointFollower>(wayPointFollower))
            qSharedPointerCast<PurepursuitWaypointFollower>(wayPointFollower)->setAdaptivePurePursuitRadiusCoefficient(coefficient);
        else
            qDebug() << "WaypointFollower has no function named setAdaptivePurePursuitRadiusCoefficient";
}

int MultiWaypointFollower::addWaypointFollower(QSharedPointer<WaypointFollower> waypointfollower)
{
    mWayPointFollowerList.append(waypointfollower);

    return (getNumberOfWaypointFollowers() - 1) ;
}

void MultiWaypointFollower::setActiveWaypointFollower(int waypointfollowerID)
{
    mWaypointFollowerID = waypointfollowerID;
}

QSharedPointer<WaypointFollower> MultiWaypointFollower::getActiveWaypointFollower()
{
    return mWayPointFollowerList[mWaypointFollowerID];
}

int MultiWaypointFollower::getNumberOfWaypointFollowers()
{
    return mWayPointFollowerList.size();
}
