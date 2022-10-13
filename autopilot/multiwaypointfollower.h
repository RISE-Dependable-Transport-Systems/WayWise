/*
 *     Copyright 2022 Rickard Häll   rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Holds multiple waypointfollowers to enable switching between routes.
 */

#ifndef MULTIWAYPOINTFOLLOWER_H
#define MULTIWAYPOINTFOLLOWER_H

#include <QObject>
#include <QList>
#include <QSharedPointer>
#include "autopilot/gotowaypointfollower.h"
#include "autopilot/purepursuitwaypointfollower.h"

class MultiWaypointFollower : public WaypointFollower
{
    Q_OBJECT
public:
    MultiWaypointFollower(QSharedPointer<WaypointFollower> waypointfollower);

    virtual bool getRepeatRoute() const override;
    virtual void setRepeatRoute(bool value) override;

    virtual const PosPoint getCurrentGoal() override;

    virtual void clearRoute() override;
    virtual void addWaypoint(const PosPoint &point) override;
    virtual void addRoute(const QList<PosPoint>& route) override;

    virtual void startFollowingRoute(bool fromBeginning) override;
    virtual bool isActive() override;
    virtual void stop() override;
    virtual void resetState() override;

    void addWaypointFollower(QSharedPointer<WaypointFollower> waypointfollower);
    void setActiveRoute(int routeID);
    QSharedPointer<WaypointFollower> getActiveWaypointFollower();
    int getNumberOfRoutes();

private:
    QList<QSharedPointer<WaypointFollower>> mWayPointFollowerList;

    int mRouteID = 0;

signals:

};

#endif // MULTIWAYPOINTFOLLOWER_H
