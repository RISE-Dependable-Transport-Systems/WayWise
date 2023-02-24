/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *               2023 Rickard Häll      rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "vehicleconnection.h"

void VehicleConnection::setWaypointFollowerConnectionLocal(const QSharedPointer<WaypointFollower> &waypointFollower) {
    qDebug() << "IMPORTANT NOTE: a connection-local WaypointFollower has been set on VehicleConnection. From now on, the VehicleConnection will only communicate to this autopilot and not to potential autopilots on the vehicle itself!";
    mWaypointFollower = waypointFollower;
}

bool VehicleConnection::hasWaypointFollowerConnectionLocal() {
    return !mWaypointFollower.isNull();
}

bool VehicleConnection::isAutopilotActive()
{
    if (!mWaypointFollower.isNull())
        return mWaypointFollower->isActive();
    else
        return isAutopilotActiveOnVehicle();
}

void VehicleConnection::restartAutopilot()
{
    if (!mWaypointFollower.isNull())
        mWaypointFollower->startFollowingRoute(true);
    else
        restartAutopilotOnVehicle();
}

void VehicleConnection::startAutopilot()
{
    if (!mWaypointFollower.isNull())
        mWaypointFollower->startFollowingRoute(false);
    else
        startAutopilotOnVehicle();
}

void VehicleConnection::pauseAutopilot()
{
    if (!mWaypointFollower.isNull())
        mWaypointFollower->stop();
    else
        pauseAutopilotOnVehicle();
}

void VehicleConnection::stopAutopilot()
{
    if (!mWaypointFollower.isNull()) {
        mWaypointFollower->stop();
        mWaypointFollower->resetState();
    } else
        stopAutopilotOnVehicle();
}

void VehicleConnection::clearRoute(int id)
{
    Q_UNUSED(id)
    if (!mWaypointFollower.isNull())
        mWaypointFollower->clearRoute();
    else
        clearRouteOnVehicle();
}

void VehicleConnection::appendToRoute(const QList<PosPoint> &route, int id)
{
    Q_UNUSED(id)
    if (!mWaypointFollower.isNull())
        mWaypointFollower->addRoute(route);
    else
        appendToRouteOnVehicle(route);
}

void VehicleConnection::setRoute(const QList<PosPoint> &route, int id) {
    clearRoute(id);
    appendToRoute(route, id);
}

QSharedPointer<VehicleState> VehicleConnection::getVehicleState() const {
    return mVehicleState;
}

QSharedPointer<Gimbal> VehicleConnection::getGimbal() const {
    return mGimbal;
}

bool VehicleConnection::hasGimbal() const {
    return !mGimbal.isNull();
}

void VehicleConnection::setActiveAutopilotID(int id)
{
    if (!mWaypointFollower.isNull())
        qDebug() << "Set active autopilot id not implemented for connection-local WaypointFollower";
    else
        setActiveAutopilotIDOnVehicle(id);
};
