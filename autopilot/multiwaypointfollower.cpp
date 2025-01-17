/*
 *     Copyright 2022 Rickard Häll   rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#include "multiwaypointfollower.h"
#include "communication/parameterserver.h"

MultiWaypointFollower::MultiWaypointFollower(QSharedPointer<WaypointFollower> waypointFollower)
{
    if(qSharedPointerDynamicCast<PurepursuitWaypointFollower>(waypointFollower))
        QObject::connect(qSharedPointerCast<PurepursuitWaypointFollower>(waypointFollower).get(), &PurepursuitWaypointFollower::distanceOfRouteLeft, this, &MultiWaypointFollower::receiveDistanceOfRouteLeft);

    mWaypointFollowerList.append(waypointFollower);
}

bool MultiWaypointFollower::getRepeatRoute() const
{
    return mWaypointFollowerList[mActiveWaypointFollowerID]->getRepeatRoute();
}

void MultiWaypointFollower::setRepeatRoute(bool value)
{
    mWaypointFollowerList[mActiveWaypointFollowerID]->setRepeatRoute(value);
}

const PosPoint MultiWaypointFollower::getCurrentGoal()
{
    return mWaypointFollowerList[mActiveWaypointFollowerID]->getCurrentGoal();
}

void MultiWaypointFollower::clearRoute()
{
    mWaypointFollowerList[mActiveWaypointFollowerID]->clearRoute();
}

void MultiWaypointFollower::addWaypoint(const PosPoint &point)
{
    mWaypointFollowerList[mActiveWaypointFollowerID]->addWaypoint(point);
}

void MultiWaypointFollower::addRoute(const QList<PosPoint> &route)
{
    mWaypointFollowerList[mActiveWaypointFollowerID]->addRoute(route);
}

void MultiWaypointFollower::startFollowingRoute(bool fromBeginning)
{
    mWaypointFollowerList[mActiveWaypointFollowerID]->startFollowingRoute(fromBeginning);
}

bool MultiWaypointFollower::isActive()
{
    return mWaypointFollowerList[mActiveWaypointFollowerID]->isActive();
}

void MultiWaypointFollower::stop()
{
    mWaypointFollowerList[mActiveWaypointFollowerID]->stop();
}

void MultiWaypointFollower::resetState()
{
    mWaypointFollowerList[mActiveWaypointFollowerID]->resetState();
}

double MultiWaypointFollower::getPurePursuitRadius() const
{
    if (qSharedPointerDynamicCast<PurepursuitWaypointFollower>(mWaypointFollowerList[mActiveWaypointFollowerID]).get())
        return qSharedPointerCast<PurepursuitWaypointFollower>(mWaypointFollowerList[mActiveWaypointFollowerID])->getPurePursuitRadius();
    else
        return 0;
}

void MultiWaypointFollower::setPurePursuitRadius(double value)
{
    for (const auto &waypointFollower : mWaypointFollowerList)
        if(qSharedPointerDynamicCast<PurepursuitWaypointFollower>(waypointFollower))
            qSharedPointerCast<PurepursuitWaypointFollower>(waypointFollower)->setPurePursuitRadius(value);
        else
            qDebug() << "WaypointFollower has no function named setPurePursuitRadius";
}

void MultiWaypointFollower::setAdaptivePurePursuitRadiusActive(bool adaptive)
{
    for (const auto &waypointFollower : mWaypointFollowerList)
        if(qSharedPointerDynamicCast<PurepursuitWaypointFollower>(waypointFollower))
            qSharedPointerCast<PurepursuitWaypointFollower>(waypointFollower)->setAdaptivePurePursuitRadiusActive(adaptive);
        else
            qDebug() << "WaypointFollower has no function named setAdaptivePurePursuitRadiusActive";
}

double MultiWaypointFollower::getAdaptivePurePursuitRadiusCoefficient()
{
    if (qSharedPointerDynamicCast<PurepursuitWaypointFollower>(mWaypointFollowerList[mActiveWaypointFollowerID]).get())
        return qSharedPointerCast<PurepursuitWaypointFollower>(mWaypointFollowerList[mActiveWaypointFollowerID])->getAdaptivePurePursuitRadiusCoefficient();
    else
        return 0;
}

void MultiWaypointFollower::setAdaptivePurePursuitRadiusCoefficient(double coefficient)
{
    for (const auto &waypointFollower : mWaypointFollowerList)
        if(qSharedPointerDynamicCast<PurepursuitWaypointFollower>(waypointFollower))
            qSharedPointerCast<PurepursuitWaypointFollower>(waypointFollower)->setAdaptivePurePursuitRadiusCoefficient(coefficient);
        else
            qDebug() << "WaypointFollower has no function named setAdaptivePurePursuitRadiusCoefficient";
}

int MultiWaypointFollower::addWaypointFollower(QSharedPointer<WaypointFollower> waypointFollower)
{
    if(qSharedPointerDynamicCast<PurepursuitWaypointFollower>(waypointFollower))
        QObject::connect(qSharedPointerCast<PurepursuitWaypointFollower>(waypointFollower).get(), &PurepursuitWaypointFollower::distanceOfRouteLeft, this, &MultiWaypointFollower::receiveDistanceOfRouteLeft);

    mWaypointFollowerList.append(waypointFollower);

    return (getNumberOfWaypointFollowers() - 1) ;
}

void MultiWaypointFollower::provideParametersToParameterServer()
{
    for (int i = 0; i < mWaypointFollowerList.size(); i++)
        provideParametersToParameterServer(i);
}

void MultiWaypointFollower::provideParametersToParameterServer(int waypointfollowerID)
{
    // Provide system parameters to ControlTower
    if (ParameterServer::getInstance()) {
        ParameterServer::getInstance()->provideFloatParameter("PP"+std::to_string(waypointfollowerID)+"_RADIUS", std::bind(&MultiWaypointFollower::setPurePursuitRadius, this, std::placeholders::_1),
                                                        std::bind(&MultiWaypointFollower::getPurePursuitRadius, this));
        ParameterServer::getInstance()->provideFloatParameter("PP"+std::to_string(waypointfollowerID)+"_ARC", std::bind(&MultiWaypointFollower::setAdaptivePurePursuitRadiusCoefficient, this, std::placeholders::_1),
                                                        std::bind(&MultiWaypointFollower::getAdaptivePurePursuitRadiusCoefficient, this));
    }
}

void MultiWaypointFollower::setActiveWaypointFollower(int waypointfollowerID)
{
    mActiveWaypointFollowerID = waypointfollowerID;
}

QSharedPointer<WaypointFollower> MultiWaypointFollower::getActiveWaypointFollower()
{
    return mWaypointFollowerList[mActiveWaypointFollowerID];
}

int MultiWaypointFollower::getNumberOfWaypointFollowers()
{
    return mWaypointFollowerList.size();
}

void MultiWaypointFollower::receiveDistanceOfRouteLeft(double meters)
{
    emit distanceOfRouteLeft(meters);
}

QList<PosPoint> MultiWaypointFollower::getCurrentRoute()
{
    return mWaypointFollowerList[mActiveWaypointFollowerID]->getCurrentRoute();
}
